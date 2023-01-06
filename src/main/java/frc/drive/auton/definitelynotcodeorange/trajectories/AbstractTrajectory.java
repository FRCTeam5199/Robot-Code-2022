package frc.drive.auton.definitelynotcodeorange.trajectories;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

import java.util.ArrayList;

public abstract class AbstractTrajectory {
    private TrajectoryConfig config;
    private ArrayList<Translation2d> interiorWaypoints = new ArrayList<>();
}
