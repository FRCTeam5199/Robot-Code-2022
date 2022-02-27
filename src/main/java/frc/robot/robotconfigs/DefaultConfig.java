package frc.robot.robotconfigs;

/*import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;*/

import edu.wpi.first.wpilibj.*;
import frc.ballstuff.intaking.Hopper;
import frc.ballstuff.intaking.Intake;
import frc.ballstuff.shooting.Shooter;
import frc.climber.Climber;
import frc.drive.AbstractDriveManager;
import frc.drive.auton.AutonType;
import frc.drive.auton.pointtopoint.AutonRoutines;
import frc.misc.PID;
import frc.motors.AbstractMotorController;
import frc.telemetry.imu.AbstractIMU;
import frc.vision.camera.IVision;

import java.io.File;
import java.io.IOException;
import java.util.Scanner;

/**
 * Literally dont mind me I am simply vibing I am here because it means you only have to change one value to completely
 * change robot settings (Otherwise, you would have to make 5 changes instead of 1)
 *
 * @author jojo2357
 */
public abstract class DefaultConfig {
    public static final String BOTKEY = loadEnvVariable("bottoken");
    public static final String SLACKBOTKEY = loadEnvVariable("slackbottoken");
    public static final String SLACKSOCKETKEY = loadEnvVariable("slacksockettoken");
    public final boolean DEBUG = true;
    public String AUTON_COMPLETE_NOISE = "";
    public boolean autonComplete = false;
    //Subsystems
    public boolean ENABLE_DRIVE = false;
    public boolean ENABLE_BALL_SHIFTERS = false;
    public boolean ENABLE_INTAKE = false;
    public boolean ENABLE_SHOOTER = false;
    public boolean ENABLE_HOOD_ARTICULATION = false;
    public boolean ENABLE_2020_HOPPER = false;
    public boolean ENABLE_2020_AGITATOR = false;
    public boolean ENABLE_2020_INDEXER = false;
    public boolean ENABLE_HOPPER = false;
    public boolean ENABLE_INDEXER = false;
    public boolean ENABLE_AGITATOR = false;
    public boolean ENABLE_AGITATOR_TOP = false;
    public boolean ENABLE_MUSIC = false;
    public boolean ENABLE_PDP = false;
    public boolean ENABLE_LEDS = false;
    public boolean ENABLE_TURRET = false;
    public boolean DRIVE_INVERT_LEFT = true;
    public boolean DRIVE_INVERT_RIGHT = false;
    public boolean ENABLE_MEMES = false;
    public boolean ENABLE_OVERHEAT_DETECTION = false;
    public boolean ENABLE_SHOOTER_COOLING = false;
    public boolean ENABLE_PNOOMATICS = false;
    public boolean ENABLE_INTAKE_SERVOS = false;
    public boolean ENABLE_CLIMBER = false;
    public boolean ENABLE_FLASHLIGHT = false;
    public boolean ENABLE_BUDDY_CLIMBER = false;
    public boolean ENABLE_CAMERA = false;
    public boolean ENABLE_HOOD_PISTON = false;

    //Misc
    public boolean ENABLE_VISION = false;
    public boolean USE_PHOTONVISION = true;
    public boolean ENABLE_IMU = false;
    public PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.CTREPCM;
    public PowerDistribution.ModuleType POWER_DISTRIBUTION_MODULE_TYPE = PowerDistribution.ModuleType.kCTRE;

    //SHOOTER
    public boolean SHOOTER_USE_TWO_MOTORS = true;
    public boolean SHOOTER_INVERTED = true;
    public boolean ENABLE_SHOOTER_BACKSPIN = false;
    public IVision.SupportedVision GOAL_CAMERA_TYPE = IVision.SupportedVision.PHOTON;

    //INTAKE
    public boolean ENABLE_INDEXER_AUTO_INDEX = true;
    public double INDEXER_DETECTION_CUTOFF_DISTANCE = -2;

    //UI Styles
    public AbstractDriveManager.DriveControlStyles DRIVE_STYLE = AbstractDriveManager.DriveControlStyles.STANDARD;
    public Shooter.ShootingControlStyles SHOOTER_CONTROL_STYLE = Shooter.ShootingControlStyles.STANDARD;
    public Intake.IntakeControlStyles INTAKE_CONTROL_STYLE = Intake.IntakeControlStyles.STANDARD;
    public Climber.ClimberControlStyles CLIMBER_CONTROL_STYLE = Climber.ClimberControlStyles.STANDARD;
    public Hopper.HopperControlStyles HOPPER_CONTROL_STYLE = Hopper.HopperControlStyles.STANDARD;

    //Motor Types
    public AbstractMotorController.SupportedMotors SHOOTER_MOTOR_TYPE = AbstractMotorController.SupportedMotors.TALON_FX;
    public AbstractMotorController.SupportedMotors INDEXER_MOTOR_TYPE = AbstractMotorController.SupportedMotors.VICTOR;
    public AbstractMotorController.SupportedMotors AGITATOR_MOTOR_TYPE = AbstractMotorController.SupportedMotors.VICTOR;
    public AbstractMotorController.SupportedMotors AGITATOR_TOP_MOTOR_TYPE = AbstractMotorController.SupportedMotors.VICTOR;
    public AbstractMotorController.SupportedMotors HOOD_MOTOR_TYPE = AbstractMotorController.SupportedMotors.CAN_SPARK_MAX;
    public AbstractMotorController.SupportedMotors DRIVE_MOTOR_TYPE = AbstractMotorController.SupportedMotors.TALON_FX;
    public AbstractMotorController.SupportedMotors TURRET_MOTOR_TYPE = AbstractMotorController.SupportedMotors.CAN_SPARK_MAX;
    public AbstractMotorController.SupportedMotors CLIMBER_MOTOR_TYPE = AbstractMotorController.SupportedMotors.VICTOR;
    public AbstractMotorController.SupportedMotors CLIMBER_STG2_MOTOR_TYPE = AbstractMotorController.SupportedMotors.TALON_FX;
    public AbstractMotorController.SupportedMotors INTAKE_MOTOR_TYPE = AbstractMotorController.SupportedMotors.VICTOR;
    public AbstractIMU.SupportedIMU IMU_TYPE = AbstractIMU.SupportedIMU.PIGEON;
    public AutonType AUTON_TYPE = AutonType.FOLLOW_PATH;
    public AutonRoutines DEFAULT_ROUTINE = AutonRoutines.DRIVE_OFF_INIT_LINE;
    public AbstractDriveManager.DriveBases DRIVE_BASE = AbstractDriveManager.DriveBases.STANDARD;
    //public ModuleConfiguration SWERVE_SDS_DRIVE_BASE;

    public int DRIVEBASE_SENSOR_UNITS_PER_ROTATION = 2048;//4096 if MagEncoder, built in 2048
    public double DRIVEBASE_DISTANCE_BETWEEN_WHEELS = -2; //Distance in meters between wheels
    public double DRIVEBASE_VOLTAGE_MULTIPLIER = 1;
    public double MAX_SPEED = 0; //max speed in fps - REAL IS 10(for 4in wheels)
    public double RUMBLE_TOLERANCE_FPS = 0; //The minimum value in which the controller will begin rumbling
    public double MAX_ROTATION = 0; //max rotational speed in radians per second - REAL IS 11.2(for 4in wheels)
    public double WHEEL_DIAMETER = 6;
    public double TURN_SCALE = 1;
    public double DRIVE_SCALE = 1;
    public double DRIVE_GEARING = 10 / 70.0;
    public int OVERHEAT_THRESHOLD = 80;

    public PID DRIVEBASE_PID = PID.EMPTY_PID;
    public PID SHOOTER_PID = PID.EMPTY_PID;
    public PID SHOOTER_CONST_SPEED_PID = PID.EMPTY_PID;
    public PID SHOOTER_RECOVERY_PID = SHOOTER_PID;
    public PID TURRET_PID = PID.EMPTY_PID;
    public PID HEADING_PID = PID.EMPTY_PID;
    public PID TURRET_HEADING_PID = PID.EMPTY_PID;
    public PID BACKSPIN_PID = PID.EMPTY_PID;
    public double CTRE_SENSOR_UNITS_PER_ROTATION = 2048;
    public double motorPulleySize = 0;//?;
    public double driverPulleySize = 0;//?;
    public double CAMERA_HEIGHT = 0; //Inches
    public double CAMERA_PITCH = 0; //Radians
    public double TARGET_HEIGHT = 0;//2.44; //Meters

    public double XBOX_CONTROLLER_DEADZONE = 0.07;
    public double MOTOR_SPROCKET_SIZE = 0;
    public double TURRET_SPROCKET_SIZE = 0;
    public double TURRET_GEAR_RATIO = 0;
    public double TURRET_MAX_POS = 270;
    public double TURRET_MIN_POS = 0;
    public double AUTON_TOLERANCE = 0.1;
    public double AUTO_SPEED = 3;
    public double AUTO_ROTATION_SPEED = 1;
    public String GOAL_CAM_NAME = "GoalCamera";
    public String BALL_CAM_NAME = "BallCamera";

    //Drive Motors
    public int DRIVE_LEADER_L_ID; //talon
    public int[] DRIVE_FOLLOWERS_L_IDS; //talon
    public int DRIVE_LEADER_R_ID; //talon
    public int[] DRIVE_FOLLOWERS_R_IDS; //talon
    //Swerve Drive Motors
    public int SWERVE_DRIVE_FR;
    public int SWERVE_TURN_FR;
    public int SWERVE_DRIVE_FL;
    public int SWERVE_TURN_FL;
    public int SWERVE_DRIVE_BR;
    public int SWERVE_TURN_BR;
    public int SWERVE_DRIVE_BL;
    public int SWERVE_TURN_BL;
    //Shooter Motors
    public Boolean BACKSPIN_INVERTED = false;
    public int BACKSPIN_ID;
    public int SHOOTER_LEADER_ID = 7; //talon
    public int SHOOTER_FOLLOWER_ID = 8; //talon
    //hood
    public int SHOOTER_HOOD_ID; //HD HEX motor via spark max
    public double SHOOTER_HOOD_MAX_POS;
    public double SHOOTER_HOOD_MIN_POS;
    public boolean SHOOTER_HOOD_INVERT_MOTOR;
    public boolean HOPPER_TOP_INVERT_MOTOR;
    public boolean HOPPER_INDEXER_INVERT_MOTOR;
    public boolean HOPPER_AGITATOR_INVERT_MOTOR;
    public boolean INTAKE_INVERT_MOTOR;
    public double SHOOTER_HOOD_CONTROL_SPEED = 0.5;
    public double SHOOTER_HOOD_OUT_OF_BOUNDS_SPEED = 0.3;
    public double TRENCH_FRONT_HOOD_POSITION;
    public double INITIATION_LINE_HOOD_POSITION;
    public double[][] CALIBRATED_HOOD_POSITION_ARRAY;

    public double SHOOTER_FLYWHEEL_WEIGHT_MULTIPLIER = 1;

    //turret
    public int TURRET_YAW_ID = 33; //550
    public boolean TURRET_INVERT = false;
    //hopper2020
    public int AGITATOR_MOTOR_ID = 10; //victor
    public int AGITATOR_TOPBAR_MOTOR_ID;
    public int INDEXER_MOTOR_ID = 11; //victor
    public int BREAK_BEAM_ID;
    public Boolean ENABLE_BREAK_BEAM = false;
    //intake
    public int INTAKE_MOTOR_ID = 12; //victor
    public int INTAKE_SERVO_L_ID = 0;
    public int INTAKE_SERVO_R_ID = 1;
    public int IMU_ID = 22; //pigeon
    //leds
    public int LED_STRAND_LENGTH = 60;
    public int LED_STRAND_PORT_ID = 9;
    //pdp
    public int PDP_ID = 0;

    public int XBOX_CONTROLLER_USB_SLOT = 0;
    public int FLIGHT_STICK_USB_SLOT = 1;
    public int BUTTON_PANEL_USB_SLOT = 2;

    //pnoomatics
    public int PCM_ID = 1;
    public int SHOOTER_COOLING_ID;
    public int INTAKE_IN_ID;
    public int INTAKE_OUT_ID;
    public int CLIMBER_IN_ID;
    public int CLIMBER_OUT_ID;
    public int BALL_SHIFTERS_IN_ID;
    public int BALL_SHIFTERS_OUT_ID;
    public int BUDDY_CLIMBER_LOCK_IN_ID;
    public int BUDDY_CLIMBER_LOCK_OUT_ID;

    //climber
    public int[] CLIMBER_MOTOR_IDS;
    public int CLIMBER_STG1_MOTOR_ID;
    public int CLIMBER_STG2_MOTOR_ID;

    //hood
    public int HOOD_ARTICULATOR_IN_ID;
    public int HOOD_ARTICULATOR_OUT_ID;

    /**
     * Must be one of the following: {@link I2C.Port} {@link SerialPort.Port} {@link SPI.Port}
     */
    public Object IMU_NAVX_PORT = I2C.Port.kMXP;
    public boolean PERMIT_ROUGE_INPUT_MAPPING = false;

    private static String loadEnvVariable(String filename) {
        try {
            Scanner fis = new Scanner(new File(filename + ".env"));
            return fis.nextLine();
        } catch (IOException e) {
            System.err.println("Could not load " + new File(filename + ".env"));
            return "";
        }
    }

    /**
     * Prints the enabled toggles for the loaded settings
     */
    public void printToggles() {
        System.out.println("-------------------<RobotSettings>-----------------");
        System.out.println("          Driving " + ENABLE_DRIVE);
        System.out.println("         Intaking " + ENABLE_INTAKE);
        System.out.println("         Shooting " + ENABLE_SHOOTER);
        System.out.println("          Hopping " + ENABLE_2020_HOPPER);
        System.out.println("           Vision " + ENABLE_VISION);
        System.out.println("              IMU " + ENABLE_IMU);
        System.out.println("              IMU " + IMU_TYPE.name());
        System.out.println("     Drive motors " + DRIVE_MOTOR_TYPE.name());
        System.out.println("   Shooter motors " + SHOOTER_MOTOR_TYPE.name());
        System.out.println("     Shoot with 2 " + SHOOTER_USE_TWO_MOTORS);
        System.out.println("      Drive style " + DRIVE_STYLE.name());
        System.out.println("   Drivebase Type " + DRIVE_BASE.name());
        System.out.println("      Shoot style " + SHOOTER_CONTROL_STYLE.name());
        System.out.println("     Intake style " + INTAKE_CONTROL_STYLE.name());
        System.out.println("  Auton Completed " + autonComplete);
        System.out.println("-------------------</RobotSettings>-----------------");
    }

    /**
     * Prints out "Numbers" which pertain to constants regarding the robot such as gearings, wheel sizes, etc. Not to be
     * confused with {@link #printMappings()} which prints numbers associated witd ID's and software. this is hardware
     */
    public void printNumbers() {
        System.out.println("-------------------<RobotSettings>-----------------");
        System.out.println("Drive PIDF " + DRIVEBASE_PID);
        System.out.println("Max drive speed/rotation " + MAX_SPEED + "/" + MAX_ROTATION);
        System.out.println("Turn + drive scale " + TURN_SCALE + "/" + DRIVE_SCALE);
        //System.out.println("");
        System.out.println("-------------------</RobotSettings>----------------");
    }

    /**
     * Prints out all of the id's for anything that needs an id
     */
    public void printMappings() {
        /*
        System.out.println("-------------------<RobotSettingspings>-----------------");
        System.out.println("                    Goal cam name: " + GOAL_CAM_NAME);
        System.out.println("                    Ball cam name: " + BALL_CAM_NAME);
        System.out.println(" Drive leader left id (followers): " + DRIVE_LEADER_L_ID + " (" + DRIVE_FOLLOWERS_L_IDS[0] + (DRIVE_FOLLOWERS_L_IDS.length > 1 ? ", " + DRIVE_FOLLOWERS_L_IDS[1] : "") + ")");
        System.out.println("Drive leader right id (followers): " + DRIVE_LEADER_R_ID + " (" + DRIVE_FOLLOWERS_R_IDS[0] + (DRIVE_FOLLOWERS_R_IDS.length > 1 ? ", " + DRIVE_FOLLOWERS_R_IDS[1] : "") + ")");
        System.out.println("        Shooter leader (follower): " + SHOOTER_LEADER_ID + " (" + SHOOTER_FOLLOWER_ID + ")");
        System.out.println("                       Turret yaw: " + TURRET_YAW_ID);
        System.out.println("                      Agitator id: " + AGITATOR_MOTOR_ID);
        System.out.println("                       Indexer id: " + INDEXER_MOTOR_ID);
        System.out.println("                        Intake id: " + INTAKE_MOTOR_ID);
        System.out.println("                           IMU id: " + IMU_ID);
        System.out.println("-------------------</RobotSettingspings>-----------------");
   */
    }
}