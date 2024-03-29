package frc.robot.robotconfigs.twentyone;

//import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.wpilibj.I2C;
import frc.ballstuff.intaking.Intake;
import frc.ballstuff.shooting.Shooter;
import frc.drive.AbstractDriveManager;
import frc.drive.auton.AutonType;
import frc.misc.PID;
import frc.motors.AbstractMotorController;
import frc.robot.robotconfigs.DefaultConfig;
import frc.sensors.camera.IVision;
import frc.telemetry.imu.AbstractIMU;

public class Swerve2021 extends DefaultConfig {
    public Swerve2021() {
        ENABLE_DRIVE = true;
        ENABLE_INTAKE = false;
        ENABLE_TURRET = false;
        ENABLE_SHOOTER = false;
        ENABLE_2020_HOPPER = false;
        ENABLE_2020_AGITATOR = false;
        ENABLE_2020_INDEXER = false;
        ENABLE_MUSIC = false;

        //LEDS
        ENABLE_LEDS = true;
        LED_STRAND_LENGTH = 300; //strand of 300, suggested 100
        LED_STRAND_PORT_ID = 0;

        DRIVE_INVERT_LEFT = true;
        DRIVE_INVERT_RIGHT = false;

        //Misc
        ENABLE_VISION = false;
        USE_PHOTONVISION = false;
        ENABLE_IMU = true;
        IMU_NAVX_PORT = I2C.Port.kMXP;
        IMU_ID = 22; //pigeon

        //SHOOTER
        SHOOTER_MOTOR_TYPE = AbstractMotorController.SupportedMotors.CAN_SPARK_MAX;//SupportedMotors.TALON_FX;
        SHOOTER_USE_TWO_MOTORS = true;
        SHOOTER_INVERTED = false;
        GOAL_CAMERA_TYPE = IVision.SupportedVision.LIMELIGHT;
        ENABLE_HOOD_ARTICULATION = false;
        INDEXER_DETECTION_CUTOFF_DISTANCE = 5;

        //INTAKE
        ENABLE_INDEXER_AUTO_INDEX = false;

        //UI Styles
        DRIVE_STYLE = AbstractDriveManager.DriveControlStyles.STANDARD;
        SHOOTER_CONTROL_STYLE = Shooter.ShootingControlStyles.ACCURACY_2021;//ShootingControlStyles.ACCURACY_2021;
        INTAKE_CONTROL_STYLE = Intake.IntakeControlStyles.STANDARD;
        DRIVE_MOTOR_TYPE = AbstractMotorController.SupportedMotors.TALON_FX;
        IMU_TYPE = AbstractIMU.SupportedIMU.PIGEON;
        DRIVE_BASE = AbstractDriveManager.DriveBases.SWIVEL;
        //SWERVE_SDS_DRIVE_BASE = SdsModuleConfigurations.MK3_STANDARD;

        AUTON_TYPE = AutonType.FOLLOW_PATH;

        DRIVEBASE_PID = new PID(0.0000001, 0, 0.0001);
        SHOOTER_PID = new PID(0.001, 0.0000005, 0.03, 0);//Accuracy. SPEED = new PID(0.0004, 0.0000007, 0.03, 0);
        SHOOTER_CONST_SPEED_PID = new PID(0.0001, 0.0000007, 0.05, 0);
        SHOOTER_RECOVERY_PID = SHOOTER_PID;
        TURRET_PID = new PID(0.006, 0.00001, 0.001);
        HEADING_PID = new PID(0.08, 0.000005, 0.0003);
        DRIVEBASE_SENSOR_UNITS_PER_ROTATION = 2048;//4096 if MagEncoder, built in 2048
        DRIVEBASE_DISTANCE_BETWEEN_WHEELS = 0.435991;
        MAX_SPEED = 10; //max speed in fps - REAL IS 10(for 4in wheels)
        RUMBLE_TOLERANCE_FPS = 8;
        MAX_ROTATION = 11.2; //max rotational speed in radians per second - REAL IS 11.2(for 4in wheels)
        WHEEL_DIAMETER = 4; //update: now it's used once
        TURN_SCALE = 0.7;
        DRIVE_SCALE = 1;
        DRIVE_GEARING = 10 / 60.0;

        CTRE_SENSOR_UNITS_PER_ROTATION = 2048;
        motorPulleySize = 0;//?;
        driverPulleySize = 0;//?;
        CAMERA_HEIGHT = 0; //Inches
        CAMERA_ANGLE = 0; //Radians
        TARGET_HEIGHT = 0;//2.44; //Meters

        XBOX_CONTROLLER_DEADZONE = 0.07;
        MOTOR_SPROCKET_SIZE = 1;
        TURRET_SPROCKET_SIZE = 11.1;
        TURRET_GEAR_RATIO = 7;
        TURRET_MAX_POS = 520;
        TURRET_MIN_POS = -2;
        TURRET_MOTOR_TYPE = AbstractMotorController.SupportedMotors.CAN_SPARK_MAX;
        AUTON_TOLERANCE = 0.1;
        AUTO_SPEED = 3;
        AUTO_ROTATION_SPEED = 1;
        XBOX_CONTROLLER_USB_SLOT = 0;
        FLIGHT_STICK_USB_SLOT = 1;
        BUTTON_PANEL_USB_SLOT = 2;

        GOAL_CAM_NAME = "GoalCamera";
        BALL_CAM_NAME = "BallCamera";

        //PowerDistribution
        ENABLE_PDP = true;
        PDP_ID = 0;

        //Drive Motors
        DRIVE_LEADER_L_ID = 1; //talon
        DRIVE_FOLLOWERS_L_IDS = new int[]{2}; //talon

        DRIVE_LEADER_R_ID = 3; //talon
        DRIVE_FOLLOWERS_R_IDS = new int[]{4}; //talon

        //Shooter Motors
        SHOOTER_LEADER_ID = 7; //talon
        SHOOTER_FOLLOWER_ID = 8; //talon
        SHOOTER_HOOD_ID = 32;

        //turret
        TURRET_YAW_ID = 33; //550
        //hopper2020
        AGITATOR_MOTOR_ID = 10; //victor
        INDEXER_MOTOR_ID = 11; //victor
        //intake
        INTAKE_MOTOR_ID = 12; //victor
    }
}
