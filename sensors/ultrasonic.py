""" I2C sensor interface"""
import wpilib

class Ultrasonic:
    # seconds between successive sensor ranging attempts
    sensor_ranging_period = 0.100

    def __init__(self, address, i2c_port=wpilib.I2C.Port.kMXP):
        self.address = address
        self.i2c_port = i2c_port
        self.i2c = wpilib.I2C(i2c_port, address)
        self.state = 'waiting'
        self.last_range = 0
        self.last_ranging_time = wpilib.Timer.getFPGATimestamp()

    def current_distance(self):
        return last_range

    def start_ranging(self):
        self.state = 'start_ranging'

    def update_address(self, new_address):
        if self.state == 'waiting':
            if new_address > 127:
                raise ValueError("I2C addresses must be <= 7 bits long")

            self.new_address = new_address
            self.state = 'change_address'

    def update(self):
        """
        Periodic update call for I2CXL-MaxSonar-EZ series ultrasonic sensors.
        Due to the latencies involved in sensor operation (on the order of
        tens to hundreds of ms), calls must be interleaved between robot
        main loop iterations to prevent issues with robot responsivity.
        """

        cur_time = wpilib.Timer.getFPGATimestamp()

        if self.state == 'waiting':
            if cur_time - self.last_ranging_time > self.sensor_ranging_period:
                self.start_ranging()
        elif self.state == 'start_ranging':
            failed = self.i2c.writeBulk([0x51])  # start ranging command
            if failed:
                self.state = 'error'
                raise IOError("I2C Write of Start Ranging Command failed")

            self.last_ranging_time = cur_time
            self.state = 'wait_ranging'
        elif self.state == 'wait_ranging':
            hi_byte, lo_byte = self.i2c.readOnly(2)
            if hi_byte != 0xFF:
                self.last_range = (hi_byte << 8) | lo_byte
                self.state = 'waiting'
        elif self.state == 'change_address':
            failed = self.i2c.writeBulk([0xAA, 0xA5, self.new_address])
            if failed:
                self.state = 'error'
                raise IOError("I2C Write of Address Change Command failed")

            self.i2c = wpilib.I2C(self.i2c_port, self.new_address)
            self.address = self.new_address
            self.state = 'wait_new_address'
        elif self.state == 'wait_new_address':
            failed = self.i2c.addressOnly()  # attempt to address sensor
            if not failed:
                self.state = 'waiting'
        elif self.state == 'error':
            pass
