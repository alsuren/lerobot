from __future__ import annotations

from threading import Thread
from typing import TYPE_CHECKING

import numpy as np
from teleop import Teleop
import ikpy
import ikpy.chain

from lerobot.common.robot_devices.motors.feetech import TorqueMode

from typing import TypeVar

T = TypeVar("T")


def dbg(t: T) -> T:
    """like rust's dbg!() macro"""
    print(t)
    return t


# This is relative to the zero position from examples/10_use_so100.md assuming you calibrated
# everything correctly (which I struggled with, and still don't have perfect)
INITIAL_ANGLES = np.array(
    [
        # orientation of base link, to make ikpy happy
        0,
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


def degrees_to_radians(degrees: np.ndarray) -> np.ndarray:
    return degrees * np.pi / 180


def radians_to_degrees(radians: np.ndarray) -> np.ndarray:
    return radians * 180 / np.pi


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
    teleop_start_pose: np.ndarray | None = None
    # e.g:
    # {
    #   'position': {'x': -0.04314558207988739, 'y': 1.3012554049491882, 'z': -0.047798436135053635},
    #   'orientation': {'x': -0.43354870848335736, 'y': 0.139184260084663, 'z': 0.09361725956363014, 'w': 0.8853807473787159},
    #   'move': False,
    #   'reference_frame': 'base',
    # }
    # TODO: make a type for this and contribute it upstream.
    teleop_start_message: dict | None = None

    # TODO: also make a "press to move" mode so we can put the phone down when we're tired
    # without moving the robot (like how sixdofone works).
    teleop_current_pose: np.ndarray | None = None
    teleop_current_message: dict | None = None
    chain: ikpy.chain.Chain

    # This returns a 4x4 transformation matrix given the position in space and orientation of the tip of your chain. This matrix is in homogeneous coordinates:
    #
    # matrix[:3, :3] (first 3 rows and 3 columns) is the orientation of the end effector
    # matrix[:3, 3] (first 3 rows of the last column) is the position of the end-effector
    # -- https://github.com/Phylliade/ikpy/wiki#getting-the-position-of-your-chain-aka-forward-kinematics-aka-fk
    initial_position: np.ndarray

    def __init__(self):

        # FIXME: find a way to load the links from URDF and remove the gripper *before* we construct the chain
        self.chain = ikpy.chain.Chain.from_urdf_file(
            "../SO-ARM100/URDF/SO_5DOF_ARM100_8j_URDF.SLDASM/urdf/SO_5DOF_ARM100_8j_URDF.SLDASM.urdf",
            base_elements=["Base"],
            active_links_mask=[False, True, True, True, True, True, False, False],
            name="SO-ARM100",
            # Distance from wrist roll to tip of static jaw, measured with a tape measure.
            last_link_vector=np.array([0, -0.11, 0.0]),
        )
        # We don't want the gripper to be part of the chain, so we remove it
        self.chain.links.remove(self.chain.links[-2])
        self.chain.active_links_mask = np.array(
            [False, True, True, True, True, True, False]
        )

        self.initial_position = self.chain.forward_kinematics(
            degrees_to_radians(INITIAL_ANGLES)
        )  # type: ignore - list case only happens if we say full_kinematics=True
        # full_initial_position: list[np.ndarray] = self.chain.forward_kinematics(
        #     degrees_to_radians(INITIAL_ANGLES),
        #     full_kinematics=True,
        # )
        # self.chain.plot(full_initial_position, show=True)

        # Try round-tripping
        initial_angles_inverse = self.chain.inverse_kinematics(
            self.initial_position[:3, 3],
            self.initial_position[:3, :3],
        )
        round_tripped_initial_position: np.ndarray = self.chain.forward_kinematics(
            dbg(initial_angles_inverse)
        )
        initial_angles_roundtripped = self.chain.inverse_kinematics(
            round_tripped_initial_position[:3, 3],
            round_tripped_initial_position[:3, :3],
        )
        # print(
        #     f"initial_position:\n{np.array2string(self.initial_position, floatmode='fixed')}"
        # )
        # print(
        #     f"round_tripped_initial_position:\n{np.array2string(round_tripped_initial_position, floatmode='fixed')}"
        # )
        dbg(self.initial_position - round_tripped_initial_position)
        assert np.allclose(
            dbg(self.initial_position), dbg(round_tripped_initial_position)
        )
        # assert np.allclose(self.initial_position, round_tripped_initial_position)

        print(
            f"initial_angles_inverse:\n{[f'{num:.3f}' for num in radians_to_degrees(initial_angles_inverse)]}"
        )
        print(
            f"initial_angles_roundtripped:\n{[f'{num:.3f}' for num in radians_to_degrees(initial_angles_roundtripped)]}"
        )
        print(f"INITIAL_ANGLES:\n{[f'{num:.3f}' for num in INITIAL_ANGLES]}")
        # print(f"Initial angles inverse: {initial_angles_inverse}")
        assert np.allclose(initial_angles_inverse, initial_angles_roundtripped)

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

        if self.teleop_start_pose is None:
            # this will *always* be eye(4) because we're starting from the zero position
            print(f"Setting start pose:\n{pose}")
            self.teleop_start_pose = pose
        if self.teleop_start_message is None:
            self.teleop_start_message = message

        self.teleop_current_pose = pose
        self.teleop_current_message = message

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
            if self.teleop_start_message is None or self.teleop_current_message is None:
                return INITIAL_ANGLES[1:]

            offset_x = (
                self.teleop_current_message["position"]["x"]
                - self.teleop_start_message["position"]["x"]
            )
            offset_y = (
                self.teleop_current_message["position"]["y"]
                - self.teleop_start_message["position"]["y"]
            )
            offset_z = (
                self.teleop_current_message["position"]["z"]
                - self.teleop_start_message["position"]["z"]
            )

            print()
            inverse_kinematics_angles = radians_to_degrees(
                self.chain.inverse_kinematics(
                    self.initial_position[:3, 3]
                    + np.array([offset_x, offset_y, offset_z])
                )
            )
            print(f"offset_x: {offset_x}, offset_y: {offset_y}, offset_z: {offset_z}")
            # TODO: inverse kinematics to convert pose to motor positions
            print(
                f"inverse_kinematics_angles: {[f'{num:.3f}' for num in inverse_kinematics_angles]}"
            )

            angle_differences = np.array(inverse_kinematics_angles) - INITIAL_ANGLES

            print(f"angle_differences: {[f'{num:.3f}' for num in angle_differences]}")
            return INITIAL_ANGLES[1:]

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
