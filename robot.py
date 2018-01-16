import wpilib
import constants
import swerve
import teleop
from autonomous import Autonomous

from lift.lift import RD4BLift
from lift.claw import Claw


class Robot(wpilib.IterativeRobot):
    def robotInit(self):
        constants.load_control_config()

        self.autoPositionSelect = wpilib.SendableChooser()
        self.autoPositionSelect.addDefault('Middle', 'Middle')
        self.autoPositionSelect.addObject('Left', 'Left')
        self.autoPositionSelect.addObject('Right', 'Right')

        wpilib.SmartDashboard.putData(
            'Robot Starting Position',
            self.autoPositionSelect)

        self.control_stick = wpilib.Joystick(0)
        self.drivetrain = swerve.SwerveDrive(
            constants.chassis_length,
            constants.chassis_width,
            constants.swerve_config
        )

        # self.lift = RD4BLift(...)
        # self.claw = Claw(...)

    def disabledInit(self):
        # We don't really _need_ to reload configuration in
        # every init call-- it's just useful for debugging.
        # (no need to restart robot code just to load new values)
        self.drivetrain.load_config_values()

    def disabledPeriodic(self):
        pass

    def autonomousInit(self):
        self.drivetrain.load_config_values()
        self.auto = Autonomous(
            self.autoPositionSelect.getSelected(),
            self.lift, self.claw, self.drivetrain
        )

    def autonomousPeriodic(self):
        self.auto.periodic()

    def teleopInit(self):
        self.drivetrain.load_config_values()
        constants.load_control_config()

    def teleopPeriodic(self):
        # For now: basic driving
        teleop.drive(self.control_stick, self.drivetrain)
        self.drivetrain.update_smart_dashboard()


if __name__ == "__main__":
    wpilib.run(Robot)
