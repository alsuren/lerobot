from __future__ import annotations

from threading import Thread
from typing import TYPE_CHECKING

import numpy as np

from lerobot.common.robot_devices.motors.feetech import TorqueMode
from teleop import Teleop

# This is relative to the zero position from examples/10_use_so100.md assuming you calibrated
# everything correctly (which I struggled with, and still don't have perfect)
# FIXME: why does the follower robot inch down by ~0.5cm every time I restart the program?
INITIAL_POSITION = np.array([0, 90, 90, 90, 0, 0])

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

    def read(self, data_name: str) -> NDArray[np.int64]:
        if data_name == "Torque_Enable":
            return np.array([TorqueMode.DISABLED.value])
        if data_name == "Present_Position":
            if self.teleop_start_message is None or self.teleop_current_message is None:
                return INITIAL_POSITION

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
            print(f"offset_x: {offset_x}, offset_y: {offset_y}, offset_z: {offset_z}")
            # TODO: inverse kinematics to convert pose to motor positions

            return INITIAL_POSITION

        raise NotImplementedError(f"TODO: read({data_name})")

    def write(
        self,
        data_name: str,
        values: int | float | NDArray[np.int64] | NDArray[np.float64],
        motor_names: str | list[str] | None = None,
    ) -> None:
        # We allow a bunch of writes that basically set the thing into follower mode
        if data_name == "Torque_Enable" and values == TorqueMode.DISABLED.value:
            return

        raise NotImplementedError("This is a fake motor, you can't write to it")


if TYPE_CHECKING:
    from numpy.typing import NDArray

    from lerobot.common.robot_devices.motors.utils import MotorsBus

    # Check that we implement the protocol
    _: MotorsBus = TeleopFakeMotorBus()
