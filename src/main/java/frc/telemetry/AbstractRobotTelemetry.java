package frc.telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.drive.AbstractDriveManager;
import frc.drive.DriveManagerStandard;
import frc.drive.DriveManagerSwerve;
import frc.drive.OldDriveManagerSwerve;
import frc.drive.auton.Point;
import frc.misc.ISubsystem;
import frc.misc.SubsystemStatus;
import frc.misc.UserInterface;
import frc.misc.UtilFunctions;
import frc.telemetry.imu.AbstractIMU;

import static frc.robot.Robot.robotSettings;

public abstract class AbstractRobotTelemetry implements ISubsystem {
    protected final AbstractDriveManager driver;
    public AbstractIMU imu;
    public Pose2d robotPose;
    public Translation2d robotTranslation;
    public Rotation2d robotRotation;

    public static AbstractRobotTelemetry createTelem(AbstractDriveManager driver) {
        if (driver instanceof OldDriveManagerSwerve || driver instanceof DriveManagerSwerve)
            return new RobotTelemetrySwivel(driver);
        if (driver instanceof DriveManagerStandard)
            return new RobotTelemetryStandard(driver);
        throw new IllegalArgumentException("Cannot create telem for that");
    }

    protected AbstractRobotTelemetry(AbstractDriveManager driver) {
        this.driver = driver;
        if ((this instanceof RobotTelemetrySwivel ^ (driver) instanceof OldDriveManagerSwerve) || (this instanceof RobotTelemetrySwivel ^ (driver) instanceof DriveManagerSwerve))
            throw new IllegalArgumentException("Incompatible telem and drive combo");
        addToMetaList();
        init();
    }

    @Override
    public void init() {
        //driver.resetDriveEncoders();
        if (!robotSettings.ENABLE_IMU)
            return;
        imu = AbstractIMU.createIMU(robotSettings.IMU_TYPE);
        resetOdometry();
    }

    @Override
    public SubsystemStatus getSubsystemStatus() {
        return (imu != null && imu.getSubsystemStatus() == SubsystemStatus.NOMINAL) && driver.getSubsystemStatus() == SubsystemStatus.NOMINAL ? SubsystemStatus.NOMINAL : SubsystemStatus.FAILED;
    }

    @Override
    public void updateGeneric() {
        robotTranslation = robotPose.getTranslation();
        robotRotation = robotPose.getRotation();
        UserInterface.smartDashboardPutNumber("RelYaw", imu.relativeYaw());
        UserInterface.smartDashboardPutNumber("AbsYaw", imu.absoluteYaw());
    }

    @Override
    public String getSubsystemName() {
        return "Guidance";
    }

    public void resetOdometry() {
        imu.resetOdometry();
        driver.resetDriveEncoders();
    }

    public Point getLocation() {
        return new Point(fieldX(), fieldY());
    }

    /**
     * @return the robot's X position in relation to its starting position(right positive) typically facing away from
     * opposing alliance station
     */
    public double fieldX() {
        return robotPose.getTranslation().getX();
    }

    /**
     * @return the robot's Y position in relation to its starting position(away positive) typically facing away from
     * opposing alliance station
     */
    public double fieldY() {
        return robotPose.getTranslation().getY();
    }

    /**
     * Calculates the angle in coordinate space between here and a given coordinates
     *
     * @param wayX x coord of query point
     * @param wayY y coord of query point
     * @return the angle between the heading and the point passed in, bounded by limits of {@link Math#atan2(double, double)}, so -180 to 180
     */
    public double angleFromHere(double wayX, double wayY) {
        return Math.toDegrees(Math.atan2(wayY - fieldY(), wayX - fieldX()));
    }

    /**
     * Gives the angle between the way the bot is facing and another point (bounds unknown, see {@link
     * #realHeadingError(double, double)})
     *
     * @param wayX x coord of query point
     * @param wayY y coord of query point
     * @return angle between heading and given point
     */
    public double headingError(double wayX, double wayY) {
        return angleFromHere(wayX, wayY) - imu.yawWraparoundAhead();
    }

    /**
     * Wraps the angle between prograde (straight forward) and the location of the given point on a range of -180 to
     * 180
     *
     * @param x x coord of other point
     * @param y y coord of other point
     * @return apparent angle between heading and passed coords
     * @see #headingError(double, double)
     */
    public double realHeadingError(double x, double y) {
        return UtilFunctions.mathematicalMod(headingError(x, y) + 180, 360) - 180;
    }

    /**
     * Wraps the angle between retrograde (straight backwards) and the location of the given point on a range of -180 to
     * 180
     *
     * @param x x coord of other point
     * @param y y coord of other point
     * @return apparent angle between inverted and passed coords
     * @see #headingError(double, double)
     */
    public double realRetrogradeHeadingError(double x, double y) {
        return UtilFunctions.mathematicalMod(headingError(x, y), 360) - 180;
    }
}
