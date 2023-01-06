package frc.drive.auton.definitelynotcodeorange.paths;

import edu.wpi.first.math.controller.RamseteController;

import java.util.Timer;

public abstract class AbstractAuton {
    private static Timer timer;
    private int trajectoryStartTime;
    private RamseteController controller = new RamseteController();

    public abstract void runPath();
}
