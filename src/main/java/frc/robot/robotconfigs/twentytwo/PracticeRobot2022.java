package frc.robot.robotconfigs.twentytwo;

import frc.ballstuff.intaking.Hopper;
import frc.ballstuff.shooting.Shooter;
import frc.drive.AbstractDriveManager;
import frc.drive.auton.AutonType;
import frc.misc.PID;
import frc.motors.AbstractMotorController;
import frc.robot.robotconfigs.DefaultConfig;

public class PracticeRobot2022 extends DefaultConfig {
    //Subsystems
    public PracticeRobot2022() {
        ENABLE_DRIVE = true;
        ENABLE_SHOOTER = true;
        ENABLE_HOPPER = true;
        ENABLE_INDEXER = true;
        ENABLE_AGITATOR = true;
        ENABLE_AGITATOR_TOP = false;//true;

        DRIVE_INVERT_LEFT = true;
        DRIVE_INVERT_RIGHT = false;

        //Misc
        ENABLE_VISION = false;
        USE_PHOTONVISION = false;
        ENABLE_IMU = false;

        //UI Styles
        DRIVE_STYLE = AbstractDriveManager.DriveControlStyles.STANDARD;
        SHOOTER_CONTROL_STYLE = Shooter.ShootingControlStyles.PRACTICE_2022;
        HOPPER_CONTROL_STYLE = Hopper.HopperControlStyles.PRACTICE_2022;
        DRIVE_MOTOR_TYPE = AbstractMotorController.SupportedMotors.TALON_FX;

        AUTON_TYPE = AutonType.FOLLOW_PATH;

        DRIVEBASE_PID = new PID(0.0075, 0, 0.002);
        HEADING_PID = new PID(0.08, 0.000005, 0.0003);
        DRIVEBASE_SENSOR_UNITS_PER_ROTATION = 2048;//4096 if MagEncoder, built in 2048
        DRIVEBASE_DISTANCE_BETWEEN_WHEELS = 0.524891;
        MAX_SPEED = 20; //max speed in fps
        RUMBLE_TOLERANCE_FPS = 14;
        MAX_ROTATION = 11.2; //max rotational speed in radians per second - REAL IS 11.2(for 4in wheels)
        WHEEL_DIAMETER = 4; //inches. update: now it's used once
        TURN_SCALE = 0.7;
        DRIVE_SCALE = 1;
        DRIVE_GEARING = 12 / 60.0;
        motorPulleySize = 0;//?;
        driverPulleySize = 0;//?;

        XBOX_CONTROLLER_DEADZONE = 0.07;
        MOTOR_SPROCKET_SIZE = 1;
        TURRET_SPROCKET_SIZE = 11.1;
        TURRET_GEAR_RATIO = 7;
        TURRET_MAX_POS = 270;
        TURRET_MIN_POS = 0;
        AUTON_TOLERANCE = 0.1;
        AUTO_SPEED = 3;
        AUTO_ROTATION_SPEED = 1;
        XBOX_CONTROLLER_USB_SLOT = 0;
        FLIGHT_STICK_USB_SLOT = 1;
        BUTTON_PANEL_USB_SLOT = 2;

        GOAL_CAM_NAME = "GoalCamera";
        BALL_CAM_NAME = "BallCamera";

        //PDP
        PDP_ID = 0;

        //Drive Motors
        DRIVE_LEADER_L_ID = 1; //talon
        DRIVE_FOLLOWERS_L_IDS = new int[]{2}; //talon
        DRIVE_LEADER_R_ID = 3; //talon
        DRIVE_FOLLOWERS_R_IDS = new int[]{4}; //talon

        IMU_ID = 22; //pigeon

        SHOOTER_LEADER_ID = 5; //talon
        SHOOTER_FOLLOWER_ID = 6; //talon
        SHOOTER_USE_TWO_MOTORS = true;
        SHOOTER_MOTOR_TYPE = AbstractMotorController.SupportedMotors.TALON_FX;
        SHOOTER_PID = new PID(1, 0.0000005, 0.03, 0.1);
        SHOOTER_CONST_SPEED_PID = new PID(0.0001, 0.0000007, 0.05, 0);
        SHOOTER_RECOVERY_PID = SHOOTER_PID;

        ENABLE_INDEXER_AUTO_INDEX = false;
        INDEXER_MOTOR_ID = 7;
        AGITATOR_MOTOR_ID = 10;
        AGITATOR_TOPBAR_MOTOR_ID = 9;
        AGITATOR_MOTOR_TYPE = AbstractMotorController.SupportedMotors.VICTOR;
        INDEXER_MOTOR_TYPE = AbstractMotorController.SupportedMotors.VICTOR;
        AGITATOR_TOP_MOTOR_TYPE = AbstractMotorController.SupportedMotors.VICTOR;

    }
}
