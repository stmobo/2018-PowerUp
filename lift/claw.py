import math
import numpy as np
import wpilib
from ctre.cantalon import CANTalon


class Claw:
    claw_open_time = 0.5  # time to allow for the claw to open, in seconds

    def __init__(self, talon_id, contact_sensor_channel):
        self.talon = CANTalon(talon_id)
        self.talon.changeControlMode(CANTalon.ControlMode.PercentVbus)

        self.contact_sensor = wpilib.DigitalInput(contact_sensor_channel)
        self.state = 'neutral'

    def close(self):
        if self.state != 'closed' and self.state != 'closing':
            self.state = 'closing'

    def open(self):
        if self.state != 'neutral' and self.state != 'opening':
            self.state = 'opening'
            self.open_start_time = None

    def toggle(self):
        if self.state == 'closed' or self.state == 'closing':
            self.open()
        elif self.state == 'neutral' or self.state == 'opening':
            self.close()

    def update(self):
        if self.state == 'manual_ctrl':
            self.open_start_time = None
            return
        elif self.state == 'neutral':
            self.open_start_time = None
            self.talon.set(0)
        elif self.state == 'closing':
            self.talon.set(-1.0)
            if self.contact_sensor.get():  # contact sensor pressed?
                self.state = 'closed'
        elif self.state == 'closed':
            # Use less power to the motors--
            # Maintain grip, but don't squeeze too hard
            self.talon.set(-0.25)
        elif self.state == 'opening':
            cur_time = wpilib.Timer.getFPGATimestamp()
            if self.open_start_time is None:
                self.open_start_time = cur_time

            self.talon.set(1.0)
            if cur_time - self.open_start_time > self.claw_open_time:
                self.state = 'neutral'
