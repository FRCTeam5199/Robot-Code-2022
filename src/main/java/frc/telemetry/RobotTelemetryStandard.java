package frc.telemetry;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.drive.AbstractDriveManager;
import frc.drive.DriveManagerStandard;
import frc.misc.ISubsystem;
import frc.misc.UserInterface;

import static frc.robot.Robot.robotSettings;

public class RobotTelemetryStandard extends AbstractRobotTelemetry implements ISubsystem {
    private final boolean DEBUG = false;
    private final NetworkTableEntry robotLocation = UserInterface.ROBOT_LOCATION.getEntry();
    public DifferentialDriveOdometry odometer;

    public RobotTelemetryStandard(AbstractDriveManager driver) {
        super(driver);
        if (!(driver instanceof DriveManagerStandard))
            throw new IllegalArgumentException("Wrong drive manager for this telem");
    }

    /**
     * creates pigeon, heading pid, and odometer
     */
    @Override
    public void init() {
        super.init();
        if (imu != null) {
            odometer = new DifferentialDriveOdometry(Rotation2d.fromDegrees(imu.absoluteYaw())); //getRotations should be in distance traveled since start (inches)
            robotPose = odometer.update(new Rotation2d(Units.degreesToRadians(imu.absoluteYaw())), Units.inchesToMeters(((DriveManagerStandard) driver).leaderL.getRotations()), Units.inchesToMeters(((DriveManagerStandard) driver).leaderR.getRotations()));
        }
    }

    /**
     * updates the robot orientation based on the IMU and distance traveled
     */
    @Override
    public void updateGeneric() {
        if (robotSettings.ENABLE_IMU) {
            robotPose = odometer.update(new Rotation2d(Units.degreesToRadians(imu.absoluteYaw())), Units.inchesToMeters(((DriveManagerStandard) driver).leaderL.getRotations()), Units.inchesToMeters(((DriveManagerStandard) driver).leaderR.getRotations()));
            super.updateGeneric();
            if (DEBUG && robotSettings.DEBUG) {
                robotLocation.setString("(" + odometer.getPoseMeters().getX() + ", " + odometer.getPoseMeters().getY() + ")");
            }
            //robotLocationOnField.setRobotPose(robotPose);
            //SmartDashboard.putData("Field", robotLocationOnField);
        }
    }

    /**
     * Resets all orienting to zeroes.
     */
    public void resetOdometry() {
        if (robotSettings.ENABLE_IMU) {
            odometer = new DifferentialDriveOdometry(Rotation2d.fromDegrees(imu.absoluteYaw()));
            imu.resetOdometry();
        }
        driver.resetDriveEncoders();
    }

    /**
     * @see #updateGeneric()
     */
    @Override
    public void updateTest() {
        updateGeneric();
    }

    /**
     * @see #updateGeneric()
     */
    @Override
    public void updateTeleop() {
        updateGeneric();
    }

    /**
     * does {@link #updateGeneric()}
     */
    @Override
    public void updateAuton() {
        updateGeneric();
    }

    @Override
    public void initTest() {
        resetOdometry();
    }

    @Override
    public void initTeleop() {

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
}