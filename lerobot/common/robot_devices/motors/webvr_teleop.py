from __future__ import annotations

import sys
from threading import Thread
from typing import TYPE_CHECKING

import numpy as np
from teleop import Teleop
import pytorch_kinematics as pk
import torch
import rerun as rr


from lerobot.common.robot_devices.motors.feetech import TorqueMode

from typing import TypeVar

T = TypeVar("T")
AnyTensor = TypeVar("AnyTensor", np.ndarray, torch.Tensor)


def dbg(t: T) -> T:
    """
    like rust's dbg!() macro (so you can print intermediate values without needing to assign temporary variables)

    e.g:
    >>> dbg(max(dbg(1), dbg(2)))
    1
    2
    2

    TODO: make it walk up the stack and print the line/statement that called dbg(), like rust's dbg!() macro.
    """
    print(t)
    return t


def to_rr(x):
    if isinstance(x, pk.Transform3d):
        return rr.Arrows3D(
            origins=x.transform_points(torch.tensor([[0.0, 0, 0]]))[0],
            vectors=x.transform_normals(torch.tensor([[0, 0, 0.1]]))[0],
        )
    raise NotImplementedError(f"to_rr({x}: {type(x)})")


# This is the zero position in the URDF relative to the zero position from examples/10_use_so100.md
# assuming you calibrated everything correctly (which I struggled with, and still don't have perfect)
INITIAL_ANGLES_OFFSET = np.array(
    [
        # shoulder rotation
        0,
        # shoulder lift
        # FIXME: why does the follower robot inch down by ~0.5cm every time I restart the program?
        90 + 30,
        # elbow lift
        90,
        # wrist lift
        90,
        # wrist roll
        0,
        # always 0 - represents fixed link from wrist roll to tip of static jaw
        0,
    ]
)


def degrees_to_radians(degrees: AnyTensor) -> AnyTensor:
    return degrees * np.pi / 180


def radians_to_degrees(radians: AnyTensor) -> AnyTensor:
    return radians * 180 / np.pi


def get_angles_for_target_position(
    chain: pk.SerialChain, target_position: pk.Transform3d, current_angles: torch.Tensor
) -> torch.Tensor:
    """
    returns the joint angles that will move the robot to the target position (inverse kinematics)

    All angles are assumed to be in radians, as described in the URDF.

    raises ValueError if it's not possible.
    """
    # FIXME: the URDF doesn't actually have joint limits, this becomes -π to π in each case.
    lim = torch.tensor(chain.get_joint_limits())
    # FIXME: Is this expensive? Should we cache this and just set ik.initial_configs before each ik.solve()?
    ik = pk.PseudoInverseIK(
        chain,
        max_iterations=500,
        retry_configs=current_angles.unsqueeze(0) + 0.01,
        joint_limits=lim.T,
        early_stopping_any_converged=True,
        early_stopping_no_improvement="all",
        lr=0.2,
    )
    sense_check_solution = ik.solve(target_position)

    if not sense_check_solution.converged_any:
        raise ValueError("IK failed to converge")

    return sense_check_solution.solutions[-1][0]


class TeleopFakeMotorBus:
    """
    This is a fake motor that uses the teleop library to pretend to be the leader

    TODO:
    - [ ] it is currently a close match for the FeetechMotorBus
      - [ ] make the MotorBus protocol actually reflect reality
      - [ ] See if we can have a shared TorqueMode enum?
    """

    calibration: dict[str, list[str | int]] = {}
    thread: Thread | None = None

    # 4x4 matrix representing the pose of the robot
    teleop_start_position: pk.Transform3d | None = None
    # TODO: also make a "press to move" mode so we can put the phone down when we're tired
    # without moving the robot (like how sixdofone works).
    teleop_target_position: pk.Transform3d | None = None

    chain: pk.SerialChain

    initial_position: pk.Transform3d
    current_position: pk.Transform3d

    initial_angles_urdf: torch.Tensor
    current_angles_urdf: torch.Tensor

    def __init__(self):
        # FIXME: this isn't going to play nicely with other uses of rerun in the same process.
        # Should probably create a recording or something?
        rr.init("teleop_fake_motor_bus")
        rr.connect()

        # FIXME: check this file into lerobot repo or make it configurable in the yaml?
        full_chain = pk.build_chain_from_urdf(
            open(
                "../SO-ARM100/URDF/SO_5DOF_ARM100_8j_URDF.SLDASM/urdf/SO_5DOF_ARM100_8j_URDF.SLDASM.urdf",
                mode="rb",
            ).read()
        )
        full_chain.print_tree()
        self.chain = pk.SerialChain(full_chain, "Fixed_Jaw")

        # sense check: can we round-trip the initial position?
        # FIXME: turn this into a unit test instead
        self.initial_angles_urdf = self.current_angles_urdf = torch.tensor(
            [0.0] * self.chain.n_joints
        )
        self.initial_position = self.chain.forward_kinematics(self.current_angles_urdf)
        print(f"self.initial_position:\n{self.initial_position}")

        # the angles might differ from what we put in, because the system is not fully constrained,
        # but we can make assertions about the position.
        round_tripped_angles = get_angles_for_target_position(
            self.chain,
            target_position=self.initial_position,
            current_angles=torch.tensor(self.current_angles_urdf),
        )
        round_tripped_position = self.chain.forward_kinematics(round_tripped_angles)

        print(f"round_tripped_position:\n{round_tripped_position}")
        assert torch.allclose(
            self.initial_position.get_matrix(),
            round_tripped_position.get_matrix(),
            atol=0.01,
        )

    def connect(self) -> None:
        print("Connecting to Teleop")
        self.thread = Thread(target=self._run)
        self.thread.start()

    def _run(self) -> None:
        teleop = Teleop()
        teleop.subscribe(self._teleop_callback)
        print("Starting teleop")
        teleop.run()

    def _teleop_callback(self, pose: np.ndarray, message: dict) -> None:
        position = pk.Transform3d(
            pos=torch.tensor(
                [
                    message["position"]["x"],
                    message["position"]["y"],
                    message["position"]["z"],
                ]
            )
            * 0.1,
            rot=[
                message["orientation"]["w"],
                message["orientation"]["x"],
                message["orientation"]["y"],
                message["orientation"]["z"],
            ],
        )

        if self.teleop_start_position is None:
            # this will *always* be eye(4) because we're starting from the zero position

            self.teleop_start_position = position
            print(f"Setting start pose:\n{self.teleop_start_position}")

        self.teleop_target_position = position

        rr.log(
            "teleop_target_position_offset",
            to_rr(
                self.teleop_start_position.inverse().compose(
                    self.teleop_target_position
                )
            ),
        )

    def disconnect(self) -> None:
        pass

    def motor_names(self) -> None:
        raise NotImplementedError("TODO: motor_names()")

    def set_calibration(self, calibration: dict[str, list[str | int]]) -> None:
        print(f"Setting calibration to {calibration}")
        self.calibration = calibration
        pass

    def apply_calibration(
        self,
        values: np.ndarray | list,
        motor_names: list[str] | None,
    ) -> None:
        raise NotImplementedError("This is a fake motor, you can't write to it")

    def revert_calibration(self) -> None:
        raise NotImplementedError("This is a fake motor, you can't write to it")

    def read(self, data_name: str) -> np.ndarray:
        if data_name == "Torque_Enable":
            return np.array([TorqueMode.DISABLED.value])
        if data_name == "Present_Position":
            if (
                self.teleop_start_position is None
                or self.teleop_target_position is None
            ):
                return INITIAL_ANGLES_OFFSET + radians_to_degrees(
                    np.array(list(self.current_angles_urdf) + [0])
                )

            offset = self.teleop_start_position.inverse().compose(
                self.teleop_target_position
            )

            target_position = self.initial_position.compose(
                # FIXME: invert the rotation of initial position before applying the teleop offset
                # and then re-apply it after, so we're not teleoperating upside down
                offset
            )

            target_position = pk.Transform3d(pos=target_position.get_matrix()[:, :3, 3])

            rr.log("target_robot_position", to_rr(target_position))
            try:
                inverse_kinematics_angles = get_angles_for_target_position(
                    self.chain,
                    target_position=target_position,
                    current_angles=self.current_angles_urdf,
                )
                print(f"{inverse_kinematics_angles=}")
            except ValueError as e:
                print(f"IK failed to converge: {e}")
                inverse_kinematics_angles = self.current_angles_urdf

            for i, (name, pos) in enumerate(
                self.chain.forward_kinematics(
                    inverse_kinematics_angles, end_only=False
                ).items()
            ):
                rr.log(f"inverse/{i}-{name}-ik", to_rr(pos))

            # TODO: inverse kinematics to convert pose to motor positions
            print(
                f"inverse_kinematics_angles: {[f'{num:.3f}' for num in radians_to_degrees(inverse_kinematics_angles)]}"
            )
            difference = inverse_kinematics_angles - self.current_angles_urdf
            if torch.any(torch.abs(difference) > 5):
                print(
                    f"WARNING: large difference in angles: {difference}. Skipping update."
                )
            else:
                self.current_angles_urdf = inverse_kinematics_angles

            return INITIAL_ANGLES_OFFSET + radians_to_degrees(
                np.array(list(self.current_angles_urdf) + [0])
            )

        raise NotImplementedError(f"TODO: read({data_name})")

    def write(
        self,
        data_name: str,
        values: int | float | np.ndarray,
        motor_names: str | list[str] | None = None,
    ) -> None:
        # We allow a bunch of writes that basically set the thing into follower mode
        if data_name == "Torque_Enable" and values == TorqueMode.DISABLED.value:
            return

        raise NotImplementedError("This is a fake motor, you can't write to it")
