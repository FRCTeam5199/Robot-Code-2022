package frc.drive.auton.SwerveAuton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.drive.AbstractDriveManager;
import frc.drive.DriveManagerSwerve;
import frc.drive.auton.AbstractAutonManager;

import java.util.Objects;

import frc.ballstuff.intaking.Intake;
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
    private static SendableChooser<AutonRoutinesSwerve> myChooserSwerve;
    private AutonRoutinesSwerve chose;

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
        try{
            chose = getSendableChooser().getSelected();
        }catch (Exception e){
            chose = AutonRoutinesSwerve.ball2Taxi;
        }
        switch (chose){
            case ball3:
                shoottwo();
                break;
            case ball4:
                shootfour();
                break;
            case testPath:
                path2();
                break;
            case ball4Defensive:
                shootFourDefensive();
                break;
            case ball2Taxi:
                noPickUp();
                break;
            default:
                noPickUp();
        }
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

    public void path2(){
        if(robotSettings.autonComplete)
            currentPathPart++;
        if(currentPathPart == 0)
            robotSettings.autonComplete = drivingChild.driveWheetRot(-30, .3, 90);
    }

    public void noPickUp(){
        if(robotSettings.autonComplete) {
            drivingChild.resetWheels();
            currentPathPart++;
        }
        if(currentPathPart == 0)
            robotSettings.autonComplete = Robot.shooter.fireAmount2022(2.5, 2000, false);
        if(currentPathPart == 1) {
            //robotSettings.autonComplete = drivingChild.turnDegree(180);
            robotSettings.autonComplete = drivingChild.turnDegree(155);
        }
        if(currentPathPart == 2)
            robotSettings.autonComplete = drivingChild.driveWheetRot(-32);
    }

    public void  shoottwo (){
        if(robotSettings.autonComplete) {
            drivingChild.resetWheels();
            currentPathPart++;
        }
        if(currentPathPart == 0)
        robotSettings.autonComplete = Robot.shooter.fireAmount2022(2.5, 2100, false);
        if(currentPathPart == 1) {
            //robotSettings.autonComplete = drivingChild.turnDegree(180);
            robotSettings.autonComplete = drivingChild.turnDegree(155);
        }
        if(currentPathPart == 2)
            robotSettings.autonComplete = drivingChild.driveWheetRot(-32);
        if(currentPathPart == 3) {
            robotSettings.autonComplete = drivingChild.turnDegree(170);
        }
        if(currentPathPart == 4){
            drivingChild.aimYaw();
        }
        if(currentPathPart == 5)
            robotSettings.autonComplete = Robot.shooter.fireAmount2022(2, 2300, true);
    }

    public void  shootFourDefensive (){
        if(robotSettings.autonComplete) {
            drivingChild.resetWheels();
            currentPathPart++;
        }
        if(currentPathPart == 0)
            robotSettings.autonComplete = Robot.shooter.fireAmount2022(2.5, 2050, false);
        if(currentPathPart == 1) {
            //robotSettings.autonComplete = drivingChild.turnDegree(180);
            robotSettings.autonComplete = drivingChild.turnDegree(-130);
        }
        if(currentPathPart == 2)
            robotSettings.autonComplete = drivingChild.driveWheetRot(-22, 0.4);
        if(currentPathPart == 3) {
            robotSettings.autonComplete = drivingChild.turnDegree(-110);
        }
        if(currentPathPart == 4){
            robotSettings.autonComplete = drivingChild.driveWheetRot(-40, 0.5);
        }
        if(currentPathPart == 5)
           robotSettings.autonComplete = drivingChild.turnDegree(-70);
        if(currentPathPart == 6)
            robotSettings.autonComplete = drivingChild.aimYaw();
        if(currentPathPart == 7)
            robotSettings.autonComplete = Robot.shooter.fireAmount2022(8, 2250, true);
    }

    public void shootfour(){
        if(robotSettings.autonComplete) {
            drivingChild.resetWheels();
            currentPathPart++;
        }
        if(currentPathPart == 0)
            robotSettings.autonComplete = Robot.shooter.fireAmount2022(2.8, 2000, false);
        if(currentPathPart == 1) {
            //robotSettings.autonComplete = drivingChild.turnDegree(180);
            robotSettings.autonComplete = drivingChild.turnDegree(128);
        }
        if(currentPathPart == 2)
            robotSettings.autonComplete = drivingChild.driveWheetRot(-21);
        if(currentPathPart == 3) {
            robotSettings.autonComplete = drivingChild.turnDegree(-65);
        }
        if(currentPathPart == 4){
            robotSettings.autonComplete = drivingChild.driveWheetRot(-51);
        }
        if(currentPathPart == 5) {
            robotSettings.autonComplete = drivingChild.turnDegree(-115);
        }
        if(currentPathPart == 6){
            drivingChild.aimYaw();
        }
        if(currentPathPart == 7)
            robotSettings.autonComplete = Robot.shooter.fireAmount2022(5, 2350, true);
    }


    public static SendableChooser<AutonRoutinesSwerve> getSendableChooser() {
            return Objects.requireNonNullElseGet(myChooserSwerve, () -> {
                myChooserSwerve = new SendableChooser<>();
                for (AutonRoutinesSwerve routine : AutonRoutinesSwerve.values())
                    myChooserSwerve.addOption(routine.name(), routine);
                return myChooserSwerve;
            });
    }

    public enum AutonRoutinesSwerve {
        ball3, ball4, ball4Defensive, testPath, ball2Taxi
    }
}
