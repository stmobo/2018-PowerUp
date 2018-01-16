import math
import numpy as np
import wpilib
from ctre.cantalon import CANTalon


class RD4BLift:
    def __init__(
        self, talon_id,
        horizontal_pos, start_pos,
        analog_range, angle_range, arm_length):
        """
        Controls a Reverse Double 4-Bar lift using a Talon SRX with attached
        potentiometer.

        Args:
            talon_id (int): CAN ID of the Talon SRX actuating the RD4B.
            horizontal_pos: Analog position (in Talon native units)
                corresponding to horizontal w.r.t the robot frame.
            start_pos: Analog position (in talon native units)
                corresponding to zero height.
            analog_range: The maximum value returned from the potentiometer
                (for the Talon SRX, this is usually 1024.)
            angle_range: The maximum angle the potentiometer can read
                (does not necessarily equal the max angle the lift can attain)
            arm_length: The length between the pivot points in the lift.
                The units used for this length must match those used in
                ``set_height()`` calls.
        """

        self.talon = CANTalon(talon_id)

        self.talon.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot)
        self.talon.changeControlMode(CANTalon.ControlMode.Position)
        self.talon.setProfile(0)

        self.h_pos = horizontal_pos
        self.angle_conversion_factor = angle_range / analog_range

        self.sin_start_angle = np.sin(
            (horizontal_pos - start_pos)
            * self.angle_conversion_factor
        )

        self.arm_length = arm_length

    def get_angle(self):
        return (
            (self.h_pos - self.talon.getAnalogInRaw())
            * self.angle_conversion_factor
        )

    # Adapted from
    # https://vamfun.wordpress.com/2014/06/08/60-in-double-reverse-4-bar-linear-linkage-example/  # noqa: E501
    def get_height(self):
        return (
            2 * self.arm_length *
            (np.sin(self.get_lift_angle()) - self.sin_start_angle)
        )

    def set_height(self, target_height):
        target_angle = np.arcsin(
            (target_height / (2 * self.arm_length))
            - self.sin_start_angle
        )

        target_pos = target_angle / self.angle_conversion_factor
        self.talon.set(target_pos + self.h_pos)
