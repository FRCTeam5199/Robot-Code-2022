package frc.drive;

import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.controllers.BaseController;
import frc.controllers.ControllerEnums;
import frc.misc.InitializationFailureException;
import frc.misc.PID;
import frc.misc.SubsystemStatus;
import frc.motors.AbstractMotorController;
import frc.motors.SwerveMotorController;
import frc.selfdiagnostics.MotorDisconnectedIssue;

import java.util.Objects;

import static frc.robot.Robot.robotSettings;

/*
notes n stuff

14wide x 22long between wheels

max speed 3.6 m/s

 */
public class DriveManagerSwerve extends AbstractDriveManager {
    private static final boolean DEBUG = false;
    private final Translation2d driftOffset = new Translation2d(-0.6, 0);
    private final double trackWidth = 13.25;
    private final double trackLength = 21.5;
    public SwerveModuleState[] moduleStates;
    public SwerveMotorController driverFR, driverBR, driverBL, driverFL;
    private Translation2d frontLeftLocation = new Translation2d(-trackLength / 2 / 39.3701, trackWidth / 2 / 39.3701);
    private Translation2d frontRightLocation = new Translation2d(-trackLength / 2 / 39.3701, -trackWidth / 2 / 39.3701);
    private Translation2d backLeftLocation = new Translation2d(trackLength / 2 / 39.3701, trackWidth / 2 / 39.3701);
    private Translation2d backRightLocation = new Translation2d(trackLength / 2 / 39.3701, -trackWidth / 2 / 39.3701);
    private SwerveDriveKinematics kinematics;
    private PIDController FRpid, BRpid, BLpid, FLpid;
    private BaseController xbox;
    private CANCoder FRcoder, BRcoder, BLcoder, FLcoder;

    public DriveManagerSwerve() {
        super();
    }

    @Override
    public void init() {
        setPIDControllers();
        createDriveMotors();
        setInvert();
        xbox = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT, BaseController.Controllers.XBOX_CONTROLLER);
        setCANCoder();
        setupSteeringEncoders();
        //setSteeringPIDS(new PID(0.005, 0.0000, 0.01));
    }

    @Override
    public SubsystemStatus getSubsystemStatus() {
        if (driverFR.driver.isFailed() || driverBR.driver.isFailed() || driverFL.driver.isFailed() || driverBL.driver.isFailed() || driverFR.steering.isFailed() || driverBR.steering.isFailed() || driverFL.steering.isFailed() || driverBL.steering.isFailed())
            return SubsystemStatus.FAILED;
        return SubsystemStatus.NOMINAL;
    }

    @Override
    public void updateTest() {
        updateGeneric();
        if (robotSettings.DEBUG && DEBUG) {
            System.out.println(FRcoder.getAbsolutePosition() + " FR " + driverFR.steering.getRotations());
            System.out.println(FLcoder.getAbsolutePosition() + " FL " + driverFL.steering.getRotations());
            System.out.println(BRcoder.getAbsolutePosition() + " BR " + driverBR.steering.getRotations());
            System.out.println(BLcoder.getAbsolutePosition() + " BL " + driverBL.steering.getRotations());
            System.out.println();
            System.out.println(guidance.imu.relativeYaw());
        }
    }

    @Override
    public void updateTeleop() {
        updateGeneric();
        driveSwerve();
        if (xbox.get(ControllerEnums.XBoxButtons.LEFT_BUMPER) == ControllerEnums.ButtonStatus.DOWN) {
            guidance.imu.resetOdometry();
        }
    }

    @Override
    public void updateAuton() {


    }

    @Override
    public void initTest() {
        resetSteeringEncoders();
        setupSteeringEncoders();
    }

    @Override
    public void initTeleop() {
        setupSteeringEncoders();
        resetSteeringEncoders();
    }

    @Override
    public void initAuton() {

    }

    @Override
    public void initDisabled() {

    }

    @Override
    public void initGeneric() {

    }

    /**
     * reset steering motor encoders
     */
    private void resetSteeringEncoders() {
        driverFR.steering.resetEncoder();
        driverBR.steering.resetEncoder();
        driverFL.steering.resetEncoder();
        driverBL.steering.resetEncoder();
    }

    private void driveSwerve() {
        double forwards = xbox.get(ControllerEnums.XboxAxes.LEFT_JOY_Y) * (-1);
        double leftwards = xbox.get(ControllerEnums.XboxAxes.LEFT_JOY_X) * (1);
        double rotation = xbox.get(ControllerEnums.XboxAxes.RIGHT_JOY_X) * (-1);

        driveMPS(adjustedDrive(forwards), adjustedDrive(leftwards), adjustedRotation(rotation));
    }

    private boolean useFieldOriented() {
        return xbox.get(ControllerEnums.XboxAxes.LEFT_TRIGGER) < 0.1;
    }

    private boolean dorifto() {
        return xbox.get(ControllerEnums.XboxAxes.RIGHT_TRIGGER) > 0.1;
    }

    /**
     * Sets the drive steering
     *
     * @param FL Front left translation requested. units?
     * @param FR Front right translation requested. units?
     * @param BL Back left translation requested. units?
     * @param BR Back right translation requested. units?
     */
    private void setSteeringContinuous(double FL, double FR, double BL, double BR) {
        double FLoffset = -1, FRoffset = -1, BLoffset = 2, BRoffset = 2;

        FLpid.setSetpoint(FL + FLoffset);
        FRpid.setSetpoint(FR + FRoffset);
        BRpid.setSetpoint(BR + BRoffset);
        BLpid.setSetpoint(BL + BLoffset);

        driverFL.steering.moveAtPercent(FLpid.calculate(FLcoder.getAbsolutePosition()));
        driverFR.steering.moveAtPercent(FRpid.calculate(FRcoder.getAbsolutePosition()));
        driverBL.steering.moveAtPercent(BLpid.calculate(BLcoder.getAbsolutePosition()));
        driverBR.steering.moveAtPercent(BRpid.calculate(BRcoder.getAbsolutePosition()));
    }

    /**
     * Drives the bot in percent control mode based on inputs
     *
     * @param FL {@link #driverFL} requested drive (-3.5, 3.5)
     * @param FR {@link #driverFR} requested drive (-3.5, 3.5)
     * @param BL {@link #driverBL} requested drive (-3.5, 3.5)
     * @param BR {@link #driverBR} requested drive (-3.5, 3.5)
     */
    private void setDrive(double FL, double FR, double BL, double BR) {
        double FPS_FL = Units.metersToFeet(FL);
        double FPS_FR = Units.metersToFeet(FR);
        double FPS_BL = Units.metersToFeet(BL);
        double FPS_BR = Units.metersToFeet(BR);
        double num = 19;
        if (robotSettings.DEBUG && DEBUG) {
            System.out.println("FL: " + FL);
            System.out.println("FR: " + FR);
            System.out.println("BL: " + BL);
            System.out.println("BR: " + BR);
        }
        /*
        driverFR.driver.moveAtVelocity(FPS_FR);
        driverBR.driver.moveAtVelocity(FPS_BR);
        driverFL.driver.moveAtVelocity(FPS_FL);
        driverBL.driver.moveAtVelocity(FPS_BL);
         */

        //todo get rid of this
        double gearRatio = 28.6472 * 12;
        driverFR.driver.moveAtVoltage(adjustedDriveVoltage((FPS_FR) * gearRatio * robotSettings.DRIVE_SCALE, 0.91 / 371.0));
        driverFL.driver.moveAtVoltage(adjustedDriveVoltage((FPS_FL) * gearRatio * robotSettings.DRIVE_SCALE, 0.91 / 371.0));
        driverBR.driver.moveAtVoltage(adjustedDriveVoltage((FPS_BR) * gearRatio * robotSettings.DRIVE_SCALE, 0.91 / 371.0));
        driverBL.driver.moveAtVoltage(adjustedDriveVoltage((FPS_BL) * gearRatio * robotSettings.DRIVE_SCALE, 0.91 / 371.0));
    }

    /**
     * set steering motors to return their encoder position in degrees
     */
    private void setupSteeringEncoders() {
        //12.8:1
        driverFR.steering.setSensorToRealDistanceFactor((1 / 12.8) * 360);
        driverBR.steering.setSensorToRealDistanceFactor((1 / 12.8) * 360);
        driverFL.steering.setSensorToRealDistanceFactor((1 / 12.8) * 360);
        driverBL.steering.setSensorToRealDistanceFactor((1 / 12.8) * 360);
    }

    //TODO implement this in regard to telem
    @Override
    public void resetDriveEncoders() {
        driverFR.driver.resetEncoder();
        driverFL.driver.resetEncoder();
        driverBR.driver.resetEncoder();
        driverBL.driver.resetEncoder();
    }

    @Override
    public void setBrake(boolean brake) {
        driverFR.driver.setBrake(brake);
        driverFL.driver.setBrake(brake);
        driverBL.driver.setBrake(brake);
        driverBR.driver.setBrake(brake);
    }

    @Override
    public void driveMPS(double xMeters, double yMeters, double rotation) {
        ChassisSpeeds speeds;

        //x+ m/s forwards, y+ m/s left, omega+ rad/sec ccw
        if (useFieldOriented() && !dorifto()) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMeters, yMeters, rotation, Rotation2d.fromDegrees(-guidance.imu.relativeYaw()));
        } else if (dorifto()) {
            speeds = new ChassisSpeeds(xMeters, 0, rotation);
        } else {
            speeds = new ChassisSpeeds(xMeters, yMeters, rotation);
        }

        driveWithChassisSpeeds(speeds);
    }

    @Override
    public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
        moduleStates = kinematics.toSwerveModuleStates(speeds);

        if (xbox.get(ControllerEnums.XBoxButtons.RIGHT_BUMPER) == ControllerEnums.ButtonStatus.DOWN) {
            moduleStates = kinematics.toSwerveModuleStates(speeds, frontRightLocation);
        } else if (dorifto()) {
            double driftOffset = 3;
            double offset = trackLength / 2 / 39.3701;
            offset -= speeds.vxMetersPerSecond / driftOffset;
            System.out.println("forwards: " + speeds.vxMetersPerSecond);
            moduleStates = kinematics.toSwerveModuleStates(speeds, new Translation2d(offset, 0));
        }

        // Front left module state
        SwerveModuleState frontLeft = moduleStates[0], frontRight = moduleStates[1], backLeft = moduleStates[2], backRight = moduleStates[3];

        //try continuous here
        setSteeringContinuous(frontLeft.angle.getDegrees(), frontRight.angle.getDegrees(), backLeft.angle.getDegrees(), backRight.angle.getDegrees());
        if (DEBUG && robotSettings.DEBUG) {
            System.out.printf("%4f %4f %4f %4f \n", frontLeft.speedMetersPerSecond, frontRight.speedMetersPerSecond, backLeft.speedMetersPerSecond, backRight.speedMetersPerSecond);
        }
        setDrive(frontLeft.speedMetersPerSecond, frontRight.speedMetersPerSecond, backLeft.speedMetersPerSecond, backRight.speedMetersPerSecond);
    }

    @Override
    public void updateGeneric() {
        MotorDisconnectedIssue.handleIssue(this, driverFL.driver);
        MotorDisconnectedIssue.handleIssue(this, driverFL.steering);
        MotorDisconnectedIssue.handleIssue(this, driverBL.driver);
        MotorDisconnectedIssue.handleIssue(this, driverBL.steering);
        MotorDisconnectedIssue.handleIssue(this, driverFR.driver);
        MotorDisconnectedIssue.handleIssue(this, driverFR.steering);
        MotorDisconnectedIssue.handleIssue(this, driverBR.driver);
        MotorDisconnectedIssue.handleIssue(this, driverBR.steering);
    }

    @Override
    protected void onControlChange() {
        //pass
    }

    /**
     * Sets the pid for all steering motors
     *
     * @param pid the pid for the swerve steering motors
     * @deprecated (For now, dont use this since the PID in the motors arent continuous)
     */
    @Deprecated
    private void setSteeringPIDS(PID pid) {
        driverFR.steering.setPid(pid);
        driverBR.steering.setPid(pid);
        driverFL.steering.setPid(pid);
        driverBL.steering.setPid(pid);
    }

    /**
     * Sets the pid for all drive motors
     *
     * @param pid the pid for the swerve drive motors
     */
    private void setDrivingPIDS(PID pid) {
        driverFR.driver.setPid(pid);
        driverBR.driver.setPid(pid);
        driverFL.driver.setPid(pid);
        driverBL.driver.setPid(pid);
    }

    public SwerveModuleState[] getStates() {
        return new SwerveModuleState[]{
                driverFL.getState(), driverFR.getState(), driverBL.getState(), driverBR.getState()
        };
    }

    public SwerveDriveKinematics getKinematics() {
        Translation2d FLPos = Objects.requireNonNullElseGet(frontLeftLocation, () -> frontLeftLocation = new Translation2d(-trackLength / 2 / 39.3701, trackWidth / 2 / 39.3701));
        Translation2d FRPos = Objects.requireNonNullElseGet(frontRightLocation, () -> frontRightLocation = new Translation2d(-trackLength / 2 / 39.3701, -trackWidth / 2 / 39.3701));
        Translation2d BLPos = Objects.requireNonNullElseGet(backLeftLocation, () -> backLeftLocation = new Translation2d(trackLength / 2 / 39.3701, trackWidth / 2 / 39.3701));
        Translation2d BRPos = Objects.requireNonNullElseGet(backRightLocation, () -> backRightLocation = new Translation2d(trackLength / 2 / 39.3701, -trackWidth / 2 / 39.3701));
        return Objects.requireNonNullElseGet(kinematics, () -> kinematics = new SwerveDriveKinematics(FLPos, FRPos, BLPos, BRPos));
    }

    private void createDriveMotors() throws InitializationFailureException {
            double s2rf;
            switch (robotSettings.DRIVE_MOTOR_TYPE) {
                case CAN_SPARK_MAX: {
                            driverFR = new SwerveMotorController(robotSettings.SWERVE_DRIVE_FR, AbstractMotorController.SupportedMotors.CAN_SPARK_MAX, robotSettings.SWERVE_TURN_FR, AbstractMotorController.SupportedMotors.CAN_SPARK_MAX);
                            driverBR = new SwerveMotorController(robotSettings.SWERVE_DRIVE_BR, AbstractMotorController.SupportedMotors.CAN_SPARK_MAX, robotSettings.SWERVE_TURN_BR, AbstractMotorController.SupportedMotors.CAN_SPARK_MAX);
                            driverBL = new SwerveMotorController(robotSettings.SWERVE_DRIVE_BL, AbstractMotorController.SupportedMotors.CAN_SPARK_MAX, robotSettings.SWERVE_TURN_BL, AbstractMotorController.SupportedMotors.CAN_SPARK_MAX);
                            driverFL = new SwerveMotorController(robotSettings.SWERVE_DRIVE_FL, AbstractMotorController.SupportedMotors.CAN_SPARK_MAX, robotSettings.SWERVE_TURN_FL, AbstractMotorController.SupportedMotors.CAN_SPARK_MAX);
                    //rpm <=> rps <=> gearing <=> wheel circumference
                    s2rf = robotSettings.DRIVE_GEARING * (robotSettings.WHEEL_DIAMETER * Math.PI);
                    break;
                }
                case TALON_FX: {
                            driverFR = new SwerveMotorController(robotSettings.SWERVE_DRIVE_FR, AbstractMotorController.SupportedMotors.TALON_FX, robotSettings.SWERVE_TURN_FR, AbstractMotorController.SupportedMotors.TALON_FX);
                            driverBR = new SwerveMotorController(robotSettings.SWERVE_DRIVE_BR, AbstractMotorController.SupportedMotors.TALON_FX, robotSettings.SWERVE_TURN_BR, AbstractMotorController.SupportedMotors.TALON_FX);
                            driverBL = new SwerveMotorController(robotSettings.SWERVE_DRIVE_BL, AbstractMotorController.SupportedMotors.TALON_FX, robotSettings.SWERVE_TURN_BL, AbstractMotorController.SupportedMotors.TALON_FX);
                            driverFL = new SwerveMotorController(robotSettings.SWERVE_DRIVE_FL, AbstractMotorController.SupportedMotors.TALON_FX, robotSettings.SWERVE_TURN_FL, AbstractMotorController.SupportedMotors.TALON_FX);
                    //Sens units / 100ms <=> rps <=> gearing <=> wheel circumference
                    s2rf = (10.0 / robotSettings.DRIVEBASE_SENSOR_UNITS_PER_ROTATION) * robotSettings.DRIVE_GEARING * (robotSettings.WHEEL_DIAMETER * Math.PI / 12);
                    break;
                }
                default:
                    throw new InitializationFailureException("DriveManager does not have a suitible constructor for " + robotSettings.DRIVE_MOTOR_TYPE.name(), "Add an implementation in the init for drive manager");
            }
    }
    public void setInvert(){
        driverFR.driver.setBrake(true);
        driverFL.driver.setInverted(true).setBrake(true);
        driverBR.driver.setBrake(true);
        driverBL.driver.setInverted(true).setBrake(true);

        driverFR.steering.setInverted(true);
        driverBR.steering.setInverted(true);
        driverBL.steering.setInverted(true);
        driverFL.steering.setInverted(true);
    }

    public void setCANCoder() {
        FRcoder = new CANCoder(11);
        BRcoder = new CANCoder(12);
        FLcoder = new CANCoder(13);
        BLcoder = new CANCoder(14);
    }

    public void  setPIDControllers() {
        PID steeringPID = new PID(0.0035, 0.000001, 0);
        setDrivingPIDS(new PID(0.001, 0, 0.0001));

        FLpid = new PIDController(steeringPID.P, steeringPID.I, steeringPID.D);
        FRpid = new PIDController(steeringPID.P, steeringPID.I, steeringPID.D);
        BLpid = new PIDController(steeringPID.P, steeringPID.I, steeringPID.D);
        BRpid = new PIDController(steeringPID.P, steeringPID.I, steeringPID.D);
        FLpid.enableContinuousInput(-180, 180);
        FRpid.enableContinuousInput(-180, 180);
        BLpid.enableContinuousInput(-180, 180);
        BRpid.enableContinuousInput(-180, 180);
    }
}
