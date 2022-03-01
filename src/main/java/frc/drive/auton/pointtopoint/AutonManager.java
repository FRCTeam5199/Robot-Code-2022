package frc.drive.auton.pointtopoint;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import frc.ballstuff.intaking.Intake;
import frc.drive.AbstractDriveManager;
import frc.drive.DriveManagerStandard;
import frc.drive.auton.AbstractAutonManager;
import frc.drive.auton.Point;
import frc.misc.SubsystemStatus;
import frc.misc.UserInterface;
import frc.misc.UtilFunctions;
import frc.motors.SparkMotorController;
import frc.robot.Robot;
import frc.telemetry.RobotTelemetryStandard;

import static frc.robot.Robot.robotSettings;

public class AutonManager extends AbstractAutonManager {
    public final Timer timer = new Timer();
    private final PIDController ROT_PID;
    public DriveManagerStandard drivingChild;
    public AutonRoutines autonPath;
    public boolean specialActionComplete = false;
    public double yawBeforeTurn = 0, rotationOffset = 0.01;

    public AutonManager(AutonRoutines routine, AbstractDriveManager driveManager) {
        super(driveManager);
        addToMetaList();
        if (driveManager instanceof DriveManagerStandard)
            drivingChild = (DriveManagerStandard) driveManager;
        else
            throw new IllegalStateException("hi wpilib");
        autonPath = routine;
        ROT_PID = new PIDController(robotSettings.HEADING_PID.P, robotSettings.HEADING_PID.I, robotSettings.HEADING_PID.D);
        init();
    }

    @Override
    public void init() {
        super.initAuton();
    }

    @Override
    public void updateTest() {

    }

    @Override
    public void updateTeleop() {
    }

    @Override
    public void updateGeneric() {
        UserInterface.smartDashboardPutNumber("Auton Stage", autonPath.currentWaypoint);
    }

    @Override
    public void initTest() {

    }

    @Override
    public void initTeleop() {

    }

    @Override
    public void initDisabled() {

    }

    @Override
    public void initGeneric() {

    }

    //getMeters - get wheel meters traveled

    @Override
    public SubsystemStatus getSubsystemStatus() {
        return drivingChild.getSubsystemStatus() == SubsystemStatus.NOMINAL && drivingChild.guidance.getSubsystemStatus() == SubsystemStatus.NOMINAL ? SubsystemStatus.NOMINAL : SubsystemStatus.FAILED;
    }

    @Override
    public void updateAuton() {
        if (autonPath.currentWaypoint >= autonPath.WAYPOINTS.size())
            return;
        updateGeneric();
        System.out.println("Home is: " + autonPath.WAYPOINTS.get(0).LOCATION + " and im going to " + autonPath.WAYPOINTS.get(autonPath.currentWaypoint).LOCATION.subtract(autonPath.WAYPOINTS.get(0).LOCATION));
        Point point = autonPath.WAYPOINTS.get(autonPath.currentWaypoint).LOCATION.subtract(autonPath.WAYPOINTS.get(0).LOCATION);
        if (attackPoint(point, autonPath.WAYPOINTS.get(autonPath.currentWaypoint).SPEED)) {
            DriverStation.reportWarning("IN TOLERANCE", false);
            System.out.println("Special Action: " + autonPath.WAYPOINTS.get(autonPath.currentWaypoint).SPECIAL_ACTION.toString());
            switch (autonPath.WAYPOINTS.get(autonPath.currentWaypoint).SPECIAL_ACTION) {
                case NONE:
                    //litterally do nothing
                    specialActionComplete = true;
                    break;
                case AIM_AT_TARGET_TRENCH:
                    specialActionComplete = Robot.turret.aimAtTarget(-4, 10);
                    break;
                case AIM_AT_TARGET_END_TRENCH:
                    specialActionComplete = Robot.turret.aimAtTarget(-2, 23.5);
                    break;
                case ZERO_TURRET:
                    Robot.turret.turretMotor.moveAtPosition(0);
                    specialActionComplete = 1 > Math.abs(Robot.turret.turretMotor.getRotations());
                case AIM_AT_TARGET_DIRECT:
                    specialActionComplete = Robot.turret.aimAtTarget();
                    break;
                case SHOOT_ONE:
                    specialActionComplete = Robot.shooter.fireAmount(1);
                    break;
                case SHOOT_TWO:
                    specialActionComplete = Robot.shooter.fireAmount(2);
                    break;
                case SHOOT_THREE:
                    specialActionComplete = Robot.shooter.fireAmount(3);
                    break;
                case SHOOT_FOUR:
                    specialActionComplete = Robot.shooter.fireAmount(4);
                    break;
                case SHOOT_ALL:
                    specialActionComplete = Robot.shooter.fireAmount(5);
                    break;
                case SHOOT_ALL_TIMED:
                    specialActionComplete = Robot.shooter.fireTimed(3);
                    break;
                case INTAKE_IN:
                    Robot.intake.setIntake(Intake.IntakeDirection.IN);
                    specialActionComplete = true;
                    break;
                case INTAKE_OFF:
                    Robot.intake.setIntake(Intake.IntakeDirection.OFF);
                    specialActionComplete = true;
                    break;
                case INTAKE_UP:
                    Robot.intake.deployIntake(false);
                    specialActionComplete = true;
                    break;
                case INTAKE_DOWN:
                    Robot.intake.deployIntake(true);
                    specialActionComplete = true;
                    break;
                case RESET_SHOOTER:
                    specialActionComplete = Robot.turret.resetShooter();
                    break;
                case AIM_ROBOT_AT_TARGET_PITCH:
                    specialActionComplete = drivingChild.aimAtTarget();
                    break;
                case DRIVE_180:
                    specialActionComplete = drivingChild.rotate180();
                    break;
                case SHOOT_ALL_2022_BEHIND_TARMAC:
                    specialActionComplete = Robot.shooter.fireAmount2022(3, 2300);
                    break;
                case SHOOT_ALL_2022_INSIDE_TARMAC:
                    specialActionComplete = Robot.shooter.fireAmount2022(3, 2000);
                    break;
                default:
                    throw new UnsupportedOperationException("Cringe. You're unable to use the Special Action " + autonPath.WAYPOINTS.get(autonPath.currentWaypoint).SPECIAL_ACTION.name() + " in your auton.");
            }
            if (specialActionComplete) {
                if (++autonPath.currentWaypoint < autonPath.WAYPOINTS.size()) {
                    //throw new IllegalStateException("Holy crap theres no way it worked. This is illegal");
                    Point b = autonPath.WAYPOINTS.get(autonPath.currentWaypoint).LOCATION.subtract(autonPath.WAYPOINTS.get(0).LOCATION);
                    setupNextPosition(b);
                    //attackPoint(b, autonPath.WAYPOINTS.get(autonPath.currentWaypoint).SPEED);
                    specialActionComplete = false;
                } else {
                    onFinish();
                }
            }
        }
    }

    /**
     * "Attack" (drive towards) a point on the field. Units are in meters and its scary.
     *
     * @param point the {@link Point point on the field} to attack
     * @param speed the speed at which to do it
     * @return true if point has been attacked
     */
    public boolean attackPoint(Point point, double speed) {
        UserInterface.smartDashboardPutString("Location", point.toString());
        //if (rotationOffset < robotSettings.AUTON_TOLERANCE) rotationOffset = 0.00;
        Point here = new Point(drivingChild.guidance.fieldX(), -drivingChild.guidance.fieldY());
        //System.out.println("I am at " + here + " and trying to turn " + rotationOffset);
        //System.out.println("Distance forward: " + Units.metersToInches(here.X));
        //System.out.println(DRIVING_CHILD.guidance.imu.absoluteYaw() + " " + DRIVING_CHILD.leaderL.getRotations() + " " + DRIVING_CHILD.leaderL.getSpeed());
        if (drivingChild.leaderL instanceof SparkMotorController) {
            UserInterface.smartDashboardPutNumber("AbsRotations", ((SparkMotorController) drivingChild.leaderL).getAbsoluteRotations());
        }
        UserInterface.smartDashboardPutNumber("WheelRotations", drivingChild.leaderL.getRotations());
        boolean inTolerance = here.isWithin(robotSettings.AUTON_TOLERANCE * 3, point);
        if (point.X == -9999 && point.Y == -9999)
            inTolerance = true;
        UserInterface.smartDashboardPutNumber("rotOffset", -rotationOffset);
        UserInterface.smartDashboardPutString("Current Position", here.toString());
        if (!inTolerance) {
            double x = autonPath.WAYPOINTS.get(autonPath.currentWaypoint).LOCATION.subtract(autonPath.WAYPOINTS.get(0).LOCATION).X;
            double y = autonPath.WAYPOINTS.get(autonPath.currentWaypoint).LOCATION.subtract(autonPath.WAYPOINTS.get(0).LOCATION).Y;
            double targetHeading = speed < 0 ? drivingChild.guidance.realRetrogradeHeadingError(x,y) : drivingChild.guidance.realHeadingError(x,y);

            drivingChild.drivePure(robotSettings.AUTO_SPEED * speed /** (robotSettings.INVERT_DRIVE_DIRECTION ? -1 : 0)*/, ROT_PID.calculate(targetHeading) * robotSettings.AUTO_ROTATION_SPEED);
        } else {
            drivingChild.drivePure(0, 0);
            System.out.println("In tolerance.");
            //System.out.println("Driving FPS " + 0);
        }
        return inTolerance;
    }

    /**
     * Saves a value with the current position and how far the robot has to turn to reach the next point
     *
     * @param point the {@link Point} (x, y) to go to
     */
    public void setupNextPosition(Point point) {
        yawBeforeTurn = drivingChild.guidance.imu.relativeYaw();
        rotationOffset = drivingChild.guidance.angleFromHere(point.X, point.Y);
    }

    /**
     * When the path finishes, we have flags to set, brakes to prime, and music to jam to
     */
    public void onFinish() {
        robotSettings.autonComplete = true;
        if (robotSettings.ENABLE_MUSIC && !robotSettings.AUTON_COMPLETE_NOISE.equals("")) {
            drivingChild.setBrake(true);
            Robot.chirp.loadMusic(robotSettings.AUTON_COMPLETE_NOISE);
            Robot.chirp.play();
        }
    }

    @Override
    public void initAuton() {
        robotSettings.autonComplete = false;
        if (robotSettings.ENABLE_IMU) {
            drivingChild.guidance.resetOdometry();
            drivingChild.guidance.imu.resetOdometry();
        }
        timer.stop();
        timer.reset();
        timer.start();
        autonPath.currentWaypoint = 0;
    }

    @Override
    public String getSubsystemName() {
        return "PointToPoint Auton Manager";
    }
}