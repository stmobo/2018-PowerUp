""" collection of functions used to manage autonomous control """
# from vision.visionmaster import VisionMaster
import wpilib


class Autonomous:
    """ Instantiate this from autonomousInit or robotInit. """
    UNKNOWN = 0
    LEFT = 1
    MIDDLE = 2
    RIGHT = 3

    # In this coordinate system:
    #   +X points downfield (towards opposing alliance)
    #   +Y points to the right (looking out thru the alliance station wall)
    #
    # -----------------------------
    #                       /^\    | A
    #       XXXXX            |     | l  S
    #       XXXXX            | +Y  | l  t
    #        | |                   | i  a
    #        | |        <--- +X    | a  t
    #       XXXXX                  | n  i
    #       XXXXX                  | c  o
    #                              | e  n
    # -----------------------------
    #
    # Coordinates are measured in inches.
    #
    # The human players at the portals stand near (0, 0).
    # The alliance station wall starts at (0, 48).
    # The center of the alliance station wall is at (0, 180).
    # The alliance station wall ends at (0, 312).
    # Robots with maximum frame size start with the centers of their frames
    #  between X=20 and X=22.5 (incl. bumpers)

    # The switches are 140in. away from the wall,
    # and about 144in (12ft) wide. The plates are 36in (3ft) wide.
    # (Half of a switch - half of a plate) gives us the offset to the center
    # of the plates relative to the center of the alliance station wall.
    left_switch_coords = np.array((140, 180-54))
    right_switch_coords = np.array((140, 180+54))

    target_offset_from_wall = 26  # (0.5*28) + 12 inches

    # These are just guesses.
    start_pos_left = np.array((21.25, 82.5))
    start_pos_middle = np.array((21.25, 208.5))
    start_pos_right = np.array((21.25, 277.5))


    drive_acceptable_err = 6  # inches

    def __init__(
        self, robot_location,
        lift, claw, drive
    ):
        self.robot_location = robot_location  # set on SmartDashboard
        # self.vision = VisionMaster('0.0.0.0', 2)

        self.lift = lift
        self.claw = claw
        self.drive = drive

        ds = wpilib.DriverStation.getInstance()
        field_string = ds.getGameSpecificMessage()

        self.close_switch = field_string[0]
        self.scale = field_string[1]
        self.far_switch = field_string[2]

        self.target = None
        self.start_pos = None

        if self.close_switch.capitalize() == 'L':
            self.target = left_switch_coords
        else:
            self.target = right_switch_coords

        # don't drive directly to switch fence
        # instead drive to a couple of inches away from it
        self.target[0] -= target_offset_from_wall

        if self.robot_location == 'Middle':
            self.start_pos = start_pos_middle
        elif self.robot_location == 'Left':
            self.start_pos = start_pos_left
        elif self.robot_location == 'Right':
            self.start_pos = start_pos_right

        drive_displacement = self.target - self.start_pos
        self.drive_angle = np.arctan2(
            drive_displacement[1],
            drive_displacement[0])
        self.drive_distance = np.sqrt(np.sum(drive_displacement ** 2))

        claw.close()
        lift.set_height(24)  # inches

        drive.reset_drive_position()
        self.state = 'drive'

    def periodic(self):
        """
        this is called by robotpy auto periodic
        decide
        """
        if self.state == 'drive':
            self.drive.drive_angle_distance(
                self.drive_angle, self.drive_distance)

            if self.drive.drive_err_within(self.drive_acceptable_err):
                self.drive.reset_drive_position()
                self.state = 'move_to_drop'
        elif self.state == 'move_to_drop'
            self.drive.drive_angle_distance(0, 10)  # fwd 10 in.

            if self.drive.drive_err_within(self.drive_acceptable_err):
                self.state = 'drop'
        elif self.state == 'drop':
            claw.open()
            self.state = 'end'
        # do nothing for end state
