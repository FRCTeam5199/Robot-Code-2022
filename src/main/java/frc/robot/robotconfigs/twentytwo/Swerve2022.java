package frc.robot.robotconfigs.twentytwo;

//import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.wpilibj.I2C;
import frc.ballstuff.intaking.Hopper;
import frc.ballstuff.intaking.Intake;
import frc.ballstuff.shooting.Shooter;
import frc.drive.AbstractDriveManager;
import frc.drive.auton.AutonType;
import frc.misc.PID;
import frc.motors.AbstractMotorController;
import frc.robot.robotconfigs.DefaultConfig;
import frc.sensors.camera.IVision;
import frc.telemetry.imu.AbstractIMU;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.climber.Climber.ClimberControlStyles;
import frc.sensors.camera.IVision;

public class Swerve2022 extends DefaultConfig {
    public Swerve2022() {
        ENABLE_SHOOTER_BACKSPIN = true;
        ENABLE_SHOOTER = true;
        ENABLE_DRIVE = true;
        ENABLE_INTAKE = true;
        ENABLE_TURRET = false;
        ENABLE_2020_HOPPER = false;
        ENABLE_2020_AGITATOR = false;
        ENABLE_2020_INDEXER = false;
        ENABLE_MUSIC = false;
        ENABLE_PDP = true;
        ENABLE_HOPPER = true;
        ENABLE_INDEXER = true;
        ENABLE_INDEXER_PISTON_BLOCK = true;
        ENABLE_AGITATOR = true;
        ENABLE_AGITATOR_TOP = true;
        ENABLE_PNOOMATICS = true;

        DRIVE_INVERT_LEFT = false;
        DRIVE_INVERT_RIGHT = false;

        //Misc
        ENABLE_VISION = false;
        USE_PHOTONVISION = false;
        ENABLE_IMU = true;
        IMU_NAVX_PORT = I2C.Port.kMXP;
        IMU_ID = 22; //pigeon


        //INTAKE
        INTAKE_CONTROL_STYLE = Intake.IntakeControlStyles.ROBOT_2022_COMP;
        INTAKE_MOTOR_TYPE = AbstractMotorController.SupportedMotors.TALON_FX;
        ENABLE_INDEXER_AUTO_INDEX = true;

        //UI Styles
        HOPPER_CONTROL_STYLE = Hopper.HopperControlStyles.COMP_2022;
        DRIVE_STYLE = AbstractDriveManager.DriveControlStyles.STANDARD;
        SHOOTER_CONTROL_STYLE = Shooter.ShootingControlStyles.COMP_2022;//ShootingControlStyles.ACCURACY_2021;
        DRIVE_MOTOR_TYPE = AbstractMotorController.SupportedMotors.CAN_SPARK_MAX;
        IMU_TYPE = AbstractIMU.SupportedIMU.PIGEON;
        DRIVE_BASE = AbstractDriveManager.DriveBases.SWIVEL;
        CLIMBER_CONTROL_STYLE = ClimberControlStyles.STANDARD_2022;
        //SWERVE_SDS_DRIVE_BASE = SdsModuleConfigurations.MK3_STANDARD;

        AUTON_TYPE = AutonType.SWERVE_P2P;

        DRIVEBASE_PID = new PID(0.0000001, 0, 0.0001);
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
        DRIVE_GEARING = 1/6.86;

        //shooter
        SHOOTER_LEADER_ID = 42; //talon
        SHOOTER_FOLLOWER_ID = 43; //talon
        SHOOTER_USE_TWO_MOTORS = true;
        SHOOTER_MOTOR_TYPE = AbstractMotorController.SupportedMotors.TALON_FX;
        SHOOTER_PID = new PID(0.195, 0.000055, 10, 0.048825);
        SHOOTER_CONST_SPEED_PID = SHOOTER_PID;//new PID(1.9, .0001, 70, 0.0851136);
        SHOOTER_RECOVERY_PID = SHOOTER_PID;
        SHOOTER_FLYWHEEL_WEIGHT_MULTIPLIER = 0.68852459016393442622950819672129;
        BACKSPIN_PID = new PID(0.2046, 0, 0, 0.0478708469817501169864295741694);//BACKSPIN_PID = new PID(0.1, 0, 0, 0.05);//PID(.132, 0.0, 10, 0.0461851);
        BACKSPIN_ID = 40;
        BACKSPIN_MULTIPLIER = 1.625;
        BACKSPIN_INVERTED = true;
        ENABLE_HOOD_ARTICULATION = true;
        ENABLE_HOOD_PISTON = true;
        SHOOTER_INVERTED = false;

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
        XBOX_CONTROLLER_USB_SLOT2 = 3;
        FLIGHT_STICK_USB_SLOT = 1;
        BUTTON_PANEL_USB_SLOT = 2;

        //PowerDistribution
        ENABLE_PDP = true;
        PDP_ID = 50;
        POWER_DISTRIBUTION_MODULE_TYPE = PowerDistribution.ModuleType.kRev;

        //pnoomatics
        PCM_ID = 33;
        PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;
        INTAKE_IN_ID = 3;
        INTAKE_OUT_ID = 2;


        INDEXER_BLOCK_IN_ID = 0;
        INDEXER_BLOCK_OUT_ID = 1;
        //CLIMBER_LOCK_IN_ID = 0;
        //CLIMBER_LOCK_OUT_ID = 0;
        CLIMBER_PISTON_IN_ID = 7;
        CLIMBER_PISTON_OUT_ID = 6;
        CLIMBER_PISTON2_IN_ID = 8;
        CLIMBER_PISTON2_OUT_ID = 9;
        HOOD_ARTICULATOR_IN_ID = 4;
        HOOD_ARTICULATOR_OUT_ID = 5;

        //Drive Motors
        SWERVE_DRIVE_FR = 1;
        SWERVE_TURN_FR = 2;
        SWERVE_DRIVE_FL = 7;
        SWERVE_TURN_FL = 8;
        SWERVE_DRIVE_BR = 4;
        SWERVE_TURN_BR = 3;
        SWERVE_DRIVE_BL = 6;
        SWERVE_TURN_BL = 5;

        //hopper
        ENABLE_BREAK_BEAM = true;
        BREAK_BEAM_DELAY_20ms = 1;
        INDEXER_SENSOR_ID = 4;//7;
        //INDEXER_DETECTION_CUTOFF_DISTANCE = 4.15;
        INDEXER_MOTOR_ID = 59;
        AGITATOR_MOTOR_ID = 58; //spark
        AGITATOR_TOPBAR_MOTOR_ID = 57;
        AGITATOR_MOTOR_TYPE = AbstractMotorController.SupportedMotors.VICTOR;
        INDEXER_MOTOR_TYPE = AbstractMotorController.SupportedMotors.VICTOR;
        AGITATOR_TOP_MOTOR_TYPE = AbstractMotorController.SupportedMotors.VICTOR;

        HOPPER_TOP_INVERT_MOTOR = false;
        HOPPER_AGITATOR_INVERT_MOTOR = true;
        HOPPER_INDEXER_INVERT_MOTOR = true;

        //intake
        INTAKE_MOTOR_ID = 60; //victor
        INTAKE_INVERT_MOTOR = true;

        INTAKE_MOTOR_CANBUS = "CANivore 1";
        SHOOTER_BACKSPIN_MOTOR_CANBUS = "CANivore 1";
        SHOOTER_MOTOR_CANBUS = "CANivore 1";

        // climber
        CLIMBER_MOTOR_TYPE = AbstractMotorController.SupportedMotors.TALON_FX;
        CLIMBER_MOTOR_IDS = new int[]{62, 41}; //LEFT, RIGHT. Specific order
        CLIMBER_BUTTON_LEFT_ID = 3;
        CLIMBER_BUTTON_RIGHT_ID = 2;
        CLIMBER_MOTOR_CANBUS = "CANivore 1";
        ENABLE_CLIMBER = true;
        ENABLE_CLIMBER_PISTON = true;
        ENABLE_CLIMBER_LOCK = false;
        LIMIT_SWITCH_ON_EACH_SIDE_CLIMBER = true;
        USE_TWO_CLIMBER_PISTONS = true;

        // limelight
        ENABLE_VISION = true;
        ENABLE_CAMERA = true;
        GOAL_CAM_NAME = "GoalCamera";
        BALL_CAM_NAME = "BallCamera";
        GOAL_CAMERA_TYPE = IVision.SupportedVision.LIMELIGHT;
        IS_LIMELIGHT_PITCH = false;
    }
}
