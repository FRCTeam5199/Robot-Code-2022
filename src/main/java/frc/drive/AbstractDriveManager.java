package frc.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.misc.ISubsystem;
import frc.misc.UserInterface;
import frc.robot.Robot;
import frc.telemetry.AbstractRobotTelemetry;
import frc.telemetry.RobotTelemetryStandard;

import java.util.Objects;

import static frc.robot.Robot.robotSettings;

/**
 * Chill out there is only vibing going on here, officer
 */
public abstract class AbstractDriveManager implements ISubsystem {
    protected final NetworkTableEntry driveRotMult = UserInterface.DRIVE_ROT_MULT.getEntry(),
            driveScaleMult = UserInterface.DRIVE_SCALE_MULT.getEntry();
    /**
     * I dont know where I am going, but i do know that whatever drive manager i end up in will love me
     */
    public AbstractRobotTelemetry guidance;


    /**
     * Required by {@link RobotTelemetryStandard} in order to reset position
     */
    public abstract void resetDriveEncoders();

    /**
     * Required by {@link frc.drive.auton.AbstractAutonManager} for stopping the robot on auton completion
     *
     * @param brake true to brake false to coast
     */
    public abstract void setBrake(boolean brake);

    public abstract void driveMPS(double xMeters, double yMeters, double rotation);

    public abstract void driveWithChassisSpeeds(ChassisSpeeds speeds);

    protected AbstractDriveManager() {
        init();
        addToMetaList();
        createTelem();
    }

    protected void createTelem() {
        if (Robot.robotSettings.ENABLE_IMU) {
            guidance = AbstractRobotTelemetry.createTelem(this);
            guidance.resetOdometry();
        }
    }

    @Override
    public void updateGeneric() {
        if (DriveControlStyles.getSendableChooser().getSelected() != null && robotSettings.DRIVE_STYLE != DriveControlStyles.getSendableChooser().getSelected()) {
            robotSettings.DRIVE_STYLE = DriveControlStyles.getSendableChooser().getSelected();
            onControlChange();
        }
    }

    protected abstract void onControlChange();

    public String getSubsystemName() {
        return "Drivetrain";
    }

    /**
     * Takes a -1 to 1 scaled value and returns it scaled based on the max sped
     *
     * @param input -1 to 1 drive amount
     * @return input scaled based on the bot's max speed
     */
    protected double adjustedDrive(double input) {
        return input * robotSettings.MAX_SPEED * driveScaleMult.getDouble(robotSettings.DRIVE_SCALE);
    }

    protected double adjustedDriveVoltage(double input, double mult) {
        double calc = input * mult;
        return (calc > 12) ? 12 : ((calc < -12) ? -12 : calc);
    }

    /**
     * Takes a -1 to 1 scaled value and returns it scaled based on the max turning
     *
     * @param input -1 to 1 drive amount
     * @return input scaled based on max turning
     */
    protected double adjustedRotation(double input) {
        return input * robotSettings.MAX_ROTATION * driveRotMult.getDouble(robotSettings.TURN_SCALE);
    }

    /**
     * How many times will i have to say it: vibing only
     */
    public enum DriveBases {
        STANDARD,
        SWIVEL
    }

    /**
     * See {@link DriveManagerStandard#updateTeleop() updateTeleop} for implementation of each drive style
     *
     * @author jojo2357
     */
    public enum DriveControlStyles {
        STANDARD,
        STANDARD_2022,
        BALL_SHIFTING_STANDARD,
        EXPERIMENTAL,
        MARIO_KART,
        DRUM_TIME,
        GUITAR,
        BOP_IT,
        FLIGHT_STICK,
        OPENLOOP_BALL_SHIFTING_STANDARD;

        private static SendableChooser<DriveControlStyles> myChooser;

        public static SendableChooser<DriveControlStyles> getSendableChooser() {
            return Objects.requireNonNullElseGet(myChooser, () -> {
                myChooser = new SendableChooser<>();
                for (DriveControlStyles style : DriveControlStyles.values())
                    myChooser.addOption(style.name(), style);
                return myChooser;
            });
        }
    }
}
