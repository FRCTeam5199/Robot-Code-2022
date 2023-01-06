package frc.drive.auton.definitelynotcodeorange;

import edu.wpi.first.math.util.Units;
import frc.drive.AbstractDriveManager;
import frc.misc.*;

public class AutonManager implements ISubsystem {
    protected final AbstractDriveManager DRIVING_CHILD;

    /**
     * Initializes the auton manager and stores the reference to the drivetrain object
     *
     * @param driveManager the drivetrain object created for the robot
     */
    protected AutonManager(AbstractDriveManager driveManager) {
        addToMetaList();
        DRIVING_CHILD = driveManager;
        init();
    }

    @Override
    public void init() {

    }

    @Override
    public SubsystemStatus getSubsystemStatus() {
        return null;
    }

    @Override
    public void updateTest() {

    }

    @Override
    public void updateTeleop() {

    }

    @Override
    public void updateAuton() {

    }

    @Override
    public void updateGeneric() {
        UserInterface.xPos.setDouble(Units.inchesToMeters(DRIVING_CHILD.guidance.robotPose.getX()));
        UserInterface.yPos.setDouble(Units.inchesToMeters(DRIVING_CHILD.guidance.robotPose.getY()));
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

    @Override
    public String getSubsystemName() {
        return null;
    }
}
