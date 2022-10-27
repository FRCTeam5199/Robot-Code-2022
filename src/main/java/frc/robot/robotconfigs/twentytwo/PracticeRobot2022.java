package frc.robot.robotconfigs.twentytwo;

import frc.ballstuff.intaking.Hopper;
import frc.ballstuff.intaking.Intake;
import frc.ballstuff.shooting.Shooter;
import frc.climber.Climber;
import frc.drive.AbstractDriveManager;
import frc.drive.auton.AutonType;
import frc.misc.PID;
import frc.motors.AbstractMotorController;
import frc.robot.robotconfigs.DefaultConfig;
import frc.sensors.camera.IVision;
import frc.telemetry.imu.AbstractIMU;

public class PracticeRobot2022 extends DefaultConfig {
    //Subsystems
    public PracticeRobot2022() {
        ENABLE_DRIVE = true;
        ENABLE_SHOOTER = true;
        ENABLE_HOPPER = true;
        ENABLE_INDEXER = true;
        ENABLE_AGITATOR = true;
        ENABLE_AGITATOR_TOP = true;
        ENABLE_INTAKE = true;
        ENABLE_PNOOMATICS = true;
        ENABLE_INDEXER_AUTO_INDEX = false;
        ENABLE_IMU = true;
        ENABLE_CAMERA = false;
        ENABLE_CLIMBER_LOCK = false;
        ENABLE_INDEXER_PISTON_BLOCK = false;
        ENABLE_CLIMBER =true;

        ENABLE_VISION = false;

        IMU_TYPE = AbstractIMU.SupportedIMU.PIGEON;

        DRIVE_INVERT_LEFT = true;
        DRIVE_INVERT_RIGHT = false;

        //Misc
        USE_PHOTONVISION = false;


        //UI Styles
        DRIVE_STYLE = AbstractDriveManager.DriveControlStyles.STANDARD_2022;
        SHOOTER_CONTROL_STYLE = Shooter.ShootingControlStyles.COMP_2022;
        HOPPER_CONTROL_STYLE = Hopper.HopperControlStyles.STANDARD_2022;
        CLIMBER_CONTROL_STYLE = Climber.ClimberControlStyles.STANDARD_2022;

        DRIVE_MOTOR_TYPE = AbstractMotorController.SupportedMotors.TALON_FX;
        CLIMBER_MOTOR_TYPE = AbstractMotorController.SupportedMotors.TALON_FX;
        CLIMBER_STG2_MOTOR_TYPE = AbstractMotorController.SupportedMotors.TALON_FX;

        AUTON_TYPE = AutonType.POINT_TO_POINT;
        AUTO_SPEED = 0.5;
        AUTO_ROTATION_SPEED = 0.75;
        GOAL_CAMERA_TYPE = IVision.SupportedVision.LIMELIGHT;

        DRIVEBASE_VOLTAGE_MULTIPLIER =  (1.050830889540567 / 2145); //3.4635 volts = 2825 RPM.
        DRIVEBASE_PID = new PID(0.1, 0, 0);
        HEADING_PID = new PID(0.08, 0.000005, 0.0003);
        DRIVEBASE_SENSOR_UNITS_PER_ROTATION = 2048;//4096 if MagEncoder, built in 2048
        CTRE_SENSOR_UNITS_PER_ROTATION = 2048;
        DRIVEBASE_DISTANCE_BETWEEN_WHEELS = 0.524891;
        MAX_SPEED = 18; //max speed in fps
        RUMBLE_TOLERANCE_FPS = 14;
        MAX_ROTATION = 11.2 * 1.5; //max rotational speed in radians per second - REAL IS 11.2(for 4in wheels)
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
        XBOX_CONTROLLER_USB_SLOT2 = 0;
        FLIGHT_STICK_USB_SLOT = 1;
        BUTTON_PANEL_USB_SLOT = 2;

        //Vision
        GOAL_CAM_NAME = "GoalCamera";
        BALL_CAM_NAME = "BallCamera";

        //PowerDistribution
        PDP_ID = 0;

        //Drive Motors
        DRIVE_LEADER_L_ID = 1; //talon
        DRIVE_FOLLOWERS_L_IDS = new int[]{2}; //talon
        DRIVE_LEADER_R_ID = 3; //talon
        DRIVE_FOLLOWERS_R_IDS = new int[]{4}; //talon

        IMU_ID = 18; //pigeon

        //Shooter
        SHOOTER_LEADER_ID = 5; //talon
        SHOOTER_FOLLOWER_ID = 6; //talon
        SHOOTER_USE_TWO_MOTORS = true;
        SHOOTER_MOTOR_TYPE = AbstractMotorController.SupportedMotors.TALON_FX;
        SHOOTER_PID = new PID(.09, 0.000, .02, 0.0);
        SHOOTER_PID = new PID(.09, 0.000, .02, 0.0);
        SHOOTER_CONST_SPEED_PID = new PID(1.9, .0001, 70, 0.0851136);
        SHOOTER_RECOVERY_PID = SHOOTER_PID;
        SHOOTER_FLYWHEEL_WEIGHT_MULTIPLIER = 0.68852459016393442622950819672129;
        SHOOTER_INVERTED = false;
        ENABLE_HOOD_ARTICULATION = true;
        ENABLE_HOOD_PISTON = true;
        HOOD_ARTICULATOR_IN_ID = 0;
        HOOD_ARTICULATOR_OUT_ID = 7;
        BACKSPIN_ID = 7;
        ENABLE_SHOOTER_BACKSPIN = true;
        BACKSPIN_MULTIPLIER = 1.625;
        BACKSPIN_PID = new PID(0.2046, 0, 0, 0.0478708469817501169864295741694);//BACKSPIN_PID = new PID(0.1, 0, 0, 0.05);//PID(.132, 0.0, 10, 0.0461851);

        //Hopper
        ENABLE_BREAK_BEAM = true;
        INDEXER_DETECTION_CUTOFF_DISTANCE = 4.15;
        INDEXER_MOTOR_ID = 23;
        AGITATOR_MOTOR_ID = 20;
        AGITATOR_TOPBAR_MOTOR_ID =22;
        AGITATOR_MOTOR_TYPE = AbstractMotorController.SupportedMotors.VICTOR;
        INDEXER_MOTOR_TYPE = AbstractMotorController.SupportedMotors.VICTOR;
        AGITATOR_TOP_MOTOR_TYPE = AbstractMotorController.SupportedMotors.VICTOR;
        INDEXER_SENSOR_ID = 0;
        ENABLE_INDEXER_BUTTON = true;

        //Intake
        INTAKE_MOTOR_ID = 21;
        INTAKE_CONTROL_STYLE = Intake.IntakeControlStyles.ROBOT_2022_COMP;
        INTAKE_MOTOR_TYPE = AbstractMotorController.SupportedMotors.VICTOR;
        INTAKE_IN_ID = 4;
        INTAKE_OUT_ID = 3;

        //Climber
        CLIMBER_STG1_MOTOR_ID = 12;
        CLIMBER_STG2_MOTOR_ID = 43;
        CLIMBER_PISTON_IN_ID = 5;
        CLIMBER_PISTON_OUT_ID = 2;
        CLIMBER_PISTON2_IN_ID = 6;
        CLIMBER_PISTON2_OUT_ID = 1;
        ENABLE_CLIMBER_PISTON = true;
        LIMIT_SWITCH_ON_EACH_SIDE_CLIMBER = true;
        USE_TWO_CLIMBER_PISTONS = true;
        CLIMBER_BUTTON_LEFT_ID = 2;
        CLIMBER_BUTTON_RIGHT_ID = 1;
        CLIMBER_MOTOR_IDS = new int[]{43, 12};

        //limelight
        ENABLE_VISION = true;
        ENABLE_CAMERA = true;
        GOAL_CAM_NAME = "GoalCamera";
        BALL_CAM_NAME = "BallCamera";
        GOAL_CAMERA_TYPE = IVision.SupportedVision.LIMELIGHT;
        IS_LIMELIGHT_PITCH = false;

        PCM_ID = 32;
    }
}