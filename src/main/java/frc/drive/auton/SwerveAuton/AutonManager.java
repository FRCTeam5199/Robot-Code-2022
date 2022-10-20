package frc.drive.auton.SwerveAuton;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import frc.drive.AbstractDriveManager;
import frc.drive.DriveManagerStandard;
import frc.drive.DriveManagerSwerve;
import frc.drive.auton.AbstractAutonManager;
import edu.wpi.first.wpilibj.Timer;
import java.sql.DriverManager;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import frc.ballstuff.intaking.Intake;
import frc.drive.AbstractDriveManager;
import frc.drive.DriveManagerStandard;
import frc.drive.auton.AbstractAutonManager;
import frc.misc.SubsystemStatus;
import frc.misc.UserInterface;
import frc.motors.SparkMotorController;
import frc.robot.Robot;

import static frc.robot.Robot.*;

public class AutonManager extends AbstractAutonManager {
    public final Timer timer = new Timer();
    public DriveManagerSwerve drivingChild;
    public double[] driverFLmove;
    public double yawBeforeTurn = 0, rotationOffset = 0.01;
    public double gearRatio = 6.86;
    public double wheelCircumference = 12.56637;
    public double[] coordinates = new double[2]; // x,y
    public int currentPathPart = 0;

    public AutonManager(AbstractDriveManager driveManager){
        super(driveManager);
        addToMetaList();
        if(driveManager instanceof DriveManagerSwerve){
            drivingChild = (DriveManagerSwerve)driveManager;
            System.out.println(drivingChild);
        }else {
            System.out.println("what the fuck is wrong with sterling");
        }
        initAuton();
    }

    @Override
    public void init() {
    }

    @Override
    public void initAuton() {
        robotSettings.autonComplete = false;
        drivingChild.setBrake(true);
        drivingChild.setBrakeTurngin(true);
        if (robotSettings.ENABLE_IMU) {
            drivingChild.guidance.resetOdometry();
            drivingChild.guidance.imu.resetOdometry();
        }
        timer.stop();
        timer.reset();
        timer.start();
        Robot.intake.deployIntake(true);
        Robot.intake.setIntake(Intake.IntakeDirection.IN);
    }

    @Override
    public void updateAuton(){
        Path2();
    }

    @Override
    public void updateTest() {

    }

    @Override
    public void updateTeleop() {

    }

    @Override
    public void updateGeneric() {

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

    public void Path2(){
        if(robotSettings.autonComplete)
            currentPathPart++;
        if(currentPathPart == 0)
            robotSettings.autonComplete = Robot.shooter.fireAmount2022(2, 2000, false);
        if(currentPathPart == 1)
            robotSettings.autonComplete = drivingChild.turnDegree(180);
        if(currentPathPart == 2) {
            robotSettings.autonComplete = drivingChild.driveWheetRot(-.3);
        }
    }

    public void Path1(){
        if(robotSettings.autonComplete)
            currentPathPart++;
        if(currentPathPart == 0)
        robotSettings.autonComplete = Robot.shooter.fireAmount2022(2, 2000, false);
        if(currentPathPart == 1) {
            //robotSettings.autonComplete = drivingChild.turnDegree(180);
            robotSettings.autonComplete = drivingChild.driveWheetRot(15);
        }
    }

}
