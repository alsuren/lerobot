from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from lerobot.common.robot_devices.motors.feetech import TorqueMode


class TeleopFakeMotorBus:
    """
    This is a fake motor that uses the teleop library to pretend to be the leader

    TODO:
    - [ ] it is currently a close match for the FeetechMotorBus
      - [ ] make the MotorBus protocol actually reflect reality
      - [ ] See if we can have a shared TorqueMode enum?
    """

    calibration: dict[str, list[str | int]] = {}

    def connect(self) -> None:
        pass

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
            # This is the zero position from examples/10_use_so100.md
            return np.array([0, 45, 0, 0, 0, 0])

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
