from ctre.talonsrx import TalonSRX
import numpy as np
import wpilib
import math
from collections import deque

ControlMode = TalonSRX.ControlMode
FeedbackDevice = TalonSRX.FeedbackDevice

class FlywheelLauncher:
    __active = False

    __sample_period = 0.05
    __sd_update_period = 0.20

    __left_speed_hist = deque([], 20)
    __right_speed_hist = deque([], 20)

    __left_side_speed = 0
    __right_side_speed = 0

    def __init__(self, left_id, right_id):
        self.left_motor = TalonSRX(left_id)
        self.right_motor = TalonSRX(right_id)

        self.left_motor.configSelectedFeedbackSensor(
            FeedbackDevice.QuadEncoder, 0, 0
        )
        self.left_motor.selectProfileSlot(0, 0)
        self.left_motor.setQuadraturePosition(0, 0)

        self.right_motor.configSelectedFeedbackSensor(
            FeedbackDevice.QuadEncoder, 0, 0
        )
        self.right_motor.selectProfileSlot(0, 0)
        self.right_motor.setQuadraturePosition(0, 0)

        self.__sample_timer = wpilib.Timer()
        self.__sd_timer = wpilib.Timer()
        self.__prefs = wpilib.Preferences.getInstance()

        self.__sample_timer.start()
        self.__sd_timer.start()
        self.load_config_values()

    def load_config_values(self):
        self.__left_reversed = self.__prefs.getBoolean('Flywheel: Reverse Left', False)  # noqa: E501
        self.__right_reversed = self.__prefs.getBoolean('Flywheel: Reverse Right', False)  # noqa: E501
        self.__target_speed = self.__prefs.getFloat('Flywheel: Target Native Speed', 0)  # noqa: E501

        self.left_motor.setSensorPhase(
            self.__prefs.getBoolean('Flywheel: Invert Phase Left', False)
        )

        self.right_motor.setSensorPhase(
            self.__prefs.getBoolean('Flywheel: Invert Phase Right', False)
        )

    def update_smart_dashboard(self):
        wpilib.SmartDashboard.putNumber(
            'Flywheel Left-Side Speed', self.__left_side_speed
        )

        wpilib.SmartDashboard.putNumber(
            'Flywheel Right-Side Speed', self.__right_side_speed
        )

        wpilib.SmartDashboard.putNumber(
            'Flywheel Left-Side Error', self.left_motor.getClosedLoopError(0)
        )

        wpilib.SmartDashboard.putNumber(
            'Flywheel Right-Side Error', self.right_motor.getClosedLoopError(0)
        )

        wpilib.SmartDashboard.putNumber(
            'Flywheel Left-Side Ticks',
            self.left_motor.getQuadraturePosition()
        )

        wpilib.SmartDashboard.putNumber(
            'Flywheel Right-Side Ticks',
            self.right_motor.getQuadraturePosition()
        )

        side_discrepancy = (
            abs(self.__right_side_speed)
            - abs(self.__left_side_speed)
        )

        wpilib.SmartDashboard.putNumber(
            'Flywheel Side Speed Discrepancy', side_discrepancy
        )

        tgt_discrepancy = (
            np.mean(np.abs((self.__left_side_speed, self.__right_side_speed)))
            - self.__target_speed
        )

        wpilib.SmartDashboard.putNumber(
            'Flywheel Target Speed Discrepancy', tgt_discrepancy
        )

    def update(self):
        if self.__sample_timer.hasPeriodPassed(__sample_period):
            self.__left_speed_hist.append(
                self.left_motor.getQuadratureVelocity()
            )

            self.__right_speed_hist.append(
                self.right_motor.getQuadratureVelocity()
            )

            self.__left_side_speed = np.mean(self.__left_speed_hist)
            self.__right_side_speed = np.mean(self.__right_speed_hist)

        if self.__sd_timer.hasPeriodPassed(__sd_update_period):
            self.update_smart_dashboard()

        if self.__active:
            self.left_motor.set(
                ControlMode.Velocity,
                self.__target_speed * (-1 if self.__left_reversed else 1)
            )

            self.right_motor.set(
                ControlMode.Velocity,
                self.__target_speed * (-1 if self.__right_reversed else 1)
            )
        else:
            self.left_motor.set(ControlMode.PercentOutput, 0)
            self.right_motor.set(ControlMode.PercentOutput, 0)
