package frc.drive.auton.pointtopoint;


import frc.drive.auton.Point;

import static frc.drive.auton.pointtopoint.AutonSpecialActions.NONE;

public class AutonWaypoint {
    public final Point LOCATION;
    public final double SPEED;
    public final AutonSpecialActions SPECIAL_ACTION;

    public AutonWaypoint(double x, double y, double speed, AutonSpecialActions action) {
        this(new Point(x, y), speed, action);
    }

    public AutonWaypoint(Point pos, double speed, AutonSpecialActions specialAction) {
        LOCATION = pos;
        SPEED = speed;
        SPECIAL_ACTION = specialAction;
    }

    public AutonWaypoint(double x, double y, double speed) {
        this(new Point(x, y), speed);
    }

    //----------------------------------------------------------------------------------------
//Anything below here has a default of no action.
//----------------------------------------------------------------------------------------
    public AutonWaypoint(Point pos, double speed) {
        this(pos, speed, NONE);
    }

    public AutonWaypoint(Point pos, AutonSpecialActions action) {
        this(pos, 1, action);
    }

    public AutonWaypoint(Point pos) {
        this(pos, 1);
    }

    public AutonWaypoint(AutonSpecialActions action) {
        this(
                new Point(-9999, -9999),
                1,
                action
        );
    }
}