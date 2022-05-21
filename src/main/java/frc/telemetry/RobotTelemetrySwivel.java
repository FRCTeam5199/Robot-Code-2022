package frc.telemetry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import frc.drive.*;

import static frc.robot.Robot.robotSettings;

/**
 * Telemetry specially designed to be compatible with {@link OldDriveManagerSwerve a swivel drivetrain} by using {@link
 * frc.motors.SwerveMotorController swivel controllers}
 */
public class RobotTelemetrySwivel extends AbstractRobotTelemetry {
    /**
     * The reason there cant be a universal RobotTelem. needs {@link edu.wpi.first.math.kinematics.SwerveModuleState
     * module states} in order to maintin track of current pos
     */
    private SwerveDriveOdometry odometer;

    protected RobotTelemetrySwivel(AbstractDriveManager driver) {
        super(driver);
        if (!(driver instanceof OldDriveManagerSwerve || driver instanceof DriveManagerSwerve))
            throw new IllegalArgumentException("Nope");
    }

    @Override
    public void init() {
        super.init();
        if (imu != null) {
            if(driver instanceof OldDriveManagerSwerve) {
                odometer = new SwerveDriveOdometry(((OldDriveManagerSwerve) driver).getKinematics(), Rotation2d.fromDegrees(imu.absoluteYaw()));
                robotPose = odometer.update(new Rotation2d(Units.degreesToRadians(imu.absoluteYaw())), ((OldDriveManagerSwerve) driver).getStates());
            }
            if(driver instanceof DriveManagerSwerve) {
                odometer = new SwerveDriveOdometry(((DriveManagerSwerve) driver).getKinematics(), Rotation2d.fromDegrees(imu.absoluteYaw()));
                robotPose = odometer.update(new Rotation2d(Units.degreesToRadians(imu.absoluteYaw())), ((DriveManagerSwerve) driver).getStates());
            }
        }

    }

    @Override
    public void updateGeneric() {
        if (robotSettings.ENABLE_IMU) {
            if(driver instanceof  OldDriveManagerSwerve)
                robotPose = odometer.update(new Rotation2d(Units.degreesToRadians(imu.absoluteYaw())), ((OldDriveManagerSwerve) driver).getStates());
            if(driver instanceof  DriveManagerSwerve)
                robotPose = odometer.update(new Rotation2d(Units.degreesToRadians(imu.absoluteYaw())), ((DriveManagerSwerve) driver).getStates());

            super.updateGeneric();
        }
    }

    @Override
    public void updateTest() {
        updateGeneric();
    }

    @Override
    public void updateTeleop() {
        updateGeneric();
    }

    @Override
    public void updateAuton() {
        updateGeneric();
    }

    @Override
    public void initTest() {

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
