package frc.robot.robotconfigs.twentytwo;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.ballstuff.intaking.Hopper.HopperControlStyles;
import frc.ballstuff.intaking.Intake;
import frc.ballstuff.shooting.Shooter.ShootingControlStyles;
import frc.climber.Climber.ClimberControlStyles;
import frc.drive.AbstractDriveManager;
import frc.drive.AbstractDriveManager.DriveControlStyles;
import frc.drive.auton.AutonType;
import frc.misc.PID;
import frc.robot.robotconfigs.DefaultConfig;
import frc.sensors.camera.IVision;
import frc.telemetry.imu.AbstractIMU;

import static frc.motors.AbstractMotorController.SupportedMotors;

public class CompetitionRobot2022 extends DefaultConfig {
    public CompetitionRobot2022() {
        ENABLE_DRIVE = true;
        ENABLE_DRIVE_BALL_TRACKING = false;
        ENABLE_SHOOTER = false;
        ENABLE_SHOOTER_BACKSPIN = false;
        ENABLE_HOPPER = false;
        ENABLE_INDEXER = false;
        ENABLE_INDEXER_PISTON_BLOCK = false;
        ENABLE_AGITATOR = false;
        ENABLE_AGITATOR_TOP = false;
        ENABLE_INTAKE = false;
        ENABLE_INTAKE_RUMBLE_BREAK_BEAM = false;
        ENABLE_INTAKE_RUMBLE_LIMIT_SWITCH =false;
        ENABLE_PNOOMATICS = false;
        ENABLE_INDEXER_AUTO_INDEX = false;
        ENABLE_IMU = true;
        ENABLE_CAMERA = false;
        ENABLE_HOOD_PISTON = false;
        ENABLE_HOOD_ARTICULATION = false;
        ENABLE_PDP = true;
        ENABLE_MUSIC = false;
        ENABLE_TOGGLEABLE_RING = false;
        ENABLE_ERROR_HANDLING = false;
        ENABLE_COLOR_SENSOR = false;
        ENABLE_VISION = false;
        ENABLE_CLIMBER = false;
        ENABLE_CLIMBER_PISTON = false;
        ENABLE_CLIMBER_LOCK = false;
        LIMIT_SWITCH_ON_EACH_SIDE_CLIMBER = false; //refer to CLIMBER_MOTOR_IDS in CompetitionRobot2022
        USE_TWO_CLIMBING_STAGES = false;
        USE_TWO_CLIMBER_PISTONS = false;
        ENABLE_INDEXER_BUTTON = false;

        IMU_TYPE = AbstractIMU.SupportedIMU.PIGEON;
        PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;

        HOPPER_TOP_INVERT_MOTOR = true;
        HOPPER_AGITATOR_INVERT_MOTOR = true;
        HOPPER_INDEXER_INVERT_MOTOR = true;
        INTAKE_INVERT_MOTOR = true;
        INVERT_DRIVE_DIRECTION = true;

        //Misc
        USE_PHOTONVISION = false;

        //UI Styles
        DRIVE_BASE = AbstractDriveManager.DriveBases.SWIVEL;
        DRIVE_STYLE = DriveControlStyles.STANDARD;
        SHOOTER_CONTROL_STYLE = ShootingControlStyles.COMP_2022;
        HOPPER_CONTROL_STYLE = HopperControlStyles.COMP_2022;
        CLIMBER_CONTROL_STYLE = ClimberControlStyles.STANDARD_2022;

        DRIVE_MOTOR_TYPE = SupportedMotors.CAN_SPARK_MAX;
        CLIMBER_MOTOR_TYPE = SupportedMotors.TALON_FX;
        //CLIMBER_STG2_MOTOR_TYPE = SupportedMotors.TALON_FX;

        AUTON_TYPE = AutonType.POINT_TO_POINT;
        AUTO_SPEED = .55;
        AUTO_ROTATION_SPEED = .10;
        AUTON_TOLERANCE = 0.08;
        GOAL_CAMERA_TYPE = IVision.SupportedVision.LIMELIGHT;
        IS_LIMELIGHT_PITCH = false;

        DRIVEBASE_VOLTAGE_MULTIPLIER = 3.5 * (1.050830889540567 / 2145); //3.4635 volts = 2825 RPM.
        //DRIVEBASE_PID = new PID(0.001, 0, 0, 0.00025); //new PID(0.0025, 0.0, 0.0);
        //HEADING_PID = new PID(0.01, 0, 0); new PID(0.15, 0.000, 0.02);//

        DRIVEBASE_PID = new PID(0.002, 0, 0.005, 0.00025);
        HEADING_PID = new PID(0.057, 0.000, 0.001);
        TELEOP_AIMING_PID = new PID(0.004, 0.00001, 0.0005818);
        AUTON_AIMING_PID = new PID(0.00419, 0.00001, 0.0005818);
        DRIVEBASE_SENSOR_UNITS_PER_ROTATION = 2048;//4096 if MagEncoder, built in 2048\
        CTRE_SENSOR_UNITS_PER_ROTATION = 2048;
        DRIVEBASE_DISTANCE_BETWEEN_WHEELS = 0.524891;
        MAX_SPEED = 20; //max speed in fps

        RUMBLE_TOLERANCE_FPS = 14;
        MAX_ROTATION = 11.2; //max rotational speed in radians per second - REAL IS 11.2(for 4in wheels)
        WHEEL_DIAMETER = 4; //update: now it's used once
        TURN_SCALE = 0.7;
        DRIVE_SCALE = 1;
        DRIVE_GEARING = 10 / 60.0;
        motorPulleySize = 0;//?;
        driverPulleySize = 0;//?;





        XBOX_CONTROLLER_DEADZONE = 0.07;
        MOTOR_SPROCKET_SIZE = 1;
        TURRET_SPROCKET_SIZE = 11.1;
        TURRET_GEAR_RATIO = 7;
        TURRET_MAX_POS = 270;
        TURRET_MIN_POS = 0;
        XBOX_CONTROLLER_USB_SLOT = 0;
        FLIGHT_STICK_USB_SLOT = 1;
        BUTTON_PANEL_USB_SLOT = 2;

        //Vision
        GOAL_CAM_NAME = "GoalCamera";
        BALL_CAM_NAME = "BallCamera";

        //Drive Motors
        SWERVE_DRIVE_FR = 1;
        SWERVE_TURN_FR = 2;
        SWERVE_DRIVE_FL = 7;
        SWERVE_TURN_FL = 8;
        SWERVE_DRIVE_BR = 4;
        SWERVE_TURN_BR = 3;
        SWERVE_DRIVE_BL = 6;
        SWERVE_TURN_BL = 5;

        IMU_ID = 22; //pigeon

        //Shooter
        SHOOTER_LEADER_ID = 9; //talon
        SHOOTER_FOLLOWER_ID = 10; //talon
        SHOOTER_USE_TWO_MOTORS = true;
        SHOOTER_MOTOR_TYPE = SupportedMotors.TALON_FX;
        SHOOTER_PID = new PID(0.195, 0.000055, 10, 0.048825);
        SHOOTER_CONST_SPEED_PID = SHOOTER_PID;//new PID(1.9, .0001, 70, 0.0851136);
        SHOOTER_RECOVERY_PID = SHOOTER_PID;
        SHOOTER_FLYWHEEL_WEIGHT_MULTIPLIER = 0.68852459016393442622950819672129;
        BACKSPIN_PID = new PID(0.2046, 0, 0, 0.0478708469817501169864295741694);//BACKSPIN_PID = new PID(0.1, 0, 0, 0.05);//PID(.132, 0.0, 10, 0.0461851);
        BACKSPIN_ID = 11;
        BACKSPIN_MULTIPLIER = 1.625;
        BACKSPIN_INVERTED = true;
        SHOOTER_INVERTED = false;

        //Hopper
        ENABLE_BREAK_BEAM = false;
        BREAK_BEAM_DELAY_20ms = 1;
        INDEXER_SENSOR_ID = 6;//7;
        //INDEXER_DETECTION_CUTOFF_DISTANCE = 4.15;
        INDEXER_MOTOR_ID = 59;
        AGITATOR_MOTOR_ID = 58; //spark
        AGITATOR_TOPBAR_MOTOR_ID = 40;
        AGITATOR_MOTOR_TYPE = SupportedMotors.CAN_SPARK_MAX;
        INDEXER_MOTOR_TYPE = SupportedMotors.CAN_SPARK_MAX;
        AGITATOR_TOP_MOTOR_TYPE = SupportedMotors.CAN_SPARK_MAX;

        //Intake
        INTAKE_MOTOR_ID = 60;
        INTAKE_CONTROL_STYLE = Intake.IntakeControlStyles.ROBOT_2022_COMP;
        INTAKE_MOTOR_TYPE = SupportedMotors.TALON_FX;
        INTAKE_BREAK_BEAM_ID = 5;

        //Climber
        /*CLIMBER_STG1_MOTOR_ID = 12;
        CLIMBER_STG2_MOTOR_ID = 62; //talon
         */
        CLIMBER_MOTOR_IDS = new int[]{62, 12}; //LEFT, RIGHT. Specific order
        CLIMBER_BUTTON_LEFT_ID = 3;
        CLIMBER_BUTTON_RIGHT_ID = 2;
        INTAKE_RUMBLE_L_ID = 4;
        INTAKE_RUMBLE_R_ID = 5;

        PCM_ID = 33;
        PDP_ID = 50;
        POWER_DISTRIBUTION_MODULE_TYPE = PowerDistribution.ModuleType.kRev;

        //Solenoids
        INTAKE_IN_ID = 3;
        INTAKE_OUT_ID = 2;
        INDEXER_BLOCK_IN_ID = 0;
        INDEXER_BLOCK_OUT_ID = 1;
        //CLIMBER_LOCK_IN_ID = 0;
        //CLIMBER_LOCK_OUT_ID = 0;
        CLIMBER_PISTON_IN_ID = 7;
        CLIMBER_PISTON_OUT_ID = 6;
        CLIMBER_PISTON2_IN_ID = 8; //todo change this @Vaughn
        CLIMBER_PISTON2_OUT_ID = 9; //todo change this @Vaughn
        HOOD_ARTICULATOR_IN_ID = 4;
        HOOD_ARTICULATOR_OUT_ID = 5;

        //target distance tracking
        CAMERA_HEIGHT = 31; //inches
        CAMERA_ANGLE = 35.9;//48.0; //degrees
        TARGET_HEIGHT = 104; //Our field sucks. COMP: 104.0" (in inches)
        ENABLE_SHOOTER_RPM_ARTICULATION = false;
        CALIBRATED_SHOOTER_RPM_ARRAY = new double[][]{ //hood down
                //Distance (in), RPM
                {23, 2200},
                {35.89, 2200},
                {83.63, 2400},
                {100, 3000}
        };

        DRIVE_MOTOR_CANBUS = "CANivore 1";
        SHOOTER_BACKSPIN_MOTOR_CANBUS = "CANivore 1";
        CLIMBER_MOTOR_CANBUS = "CANivore 1";
        INTAKE_MOTOR_CANBUS = "CANivore 1";
    }
}