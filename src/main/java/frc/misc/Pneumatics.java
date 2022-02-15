package frc.misc;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.climber.Climber;

import static frc.robot.Robot.robotSettings;

/**
 * I accidentally deleted this, so here we go again. Allows you to control all of the solenoids for all of your air
 * powered needs (pnoomatics)
 *
 * @author Smaltin
 */
public class Pneumatics implements ISubsystem {
    public DoubleSolenoid solenoidIntake;
    public DoubleSolenoid climberLock;
    public DoubleSolenoid ballShifter;
    public DoubleSolenoid buddyClimberLock;
    public Solenoid shooterCooling;
    public Compressor compressor;

    public Pneumatics() {
        addToMetaList();
        init();
    }

    @Override
    public void init() {
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();
        if (robotSettings.ENABLE_INTAKE && robotSettings.ENABLE_PNOOMATICS) {
            solenoidIntake = new DoubleSolenoid(robotSettings.PCM_ID, PneumaticsModuleType.CTREPCM, robotSettings.INTAKE_OUT_ID, robotSettings.INTAKE_IN_ID);
        }
        if (robotSettings.ENABLE_CLIMBER && robotSettings.ENABLE_PNOOMATICS) {
            climberLock = new DoubleSolenoid(robotSettings.PCM_ID, PneumaticsModuleType.CTREPCM, robotSettings.CLIMBER_IN_ID, robotSettings.CLIMBER_OUT_ID);
        }
        if (robotSettings.ENABLE_SHOOTER && robotSettings.ENABLE_PNOOMATICS && robotSettings.ENABLE_SHOOTER_COOLING) {
            shooterCooling = new Solenoid(robotSettings.PCM_ID, PneumaticsModuleType.CTREPCM, robotSettings.SHOOTER_COOLING_ID);
        }
        if (robotSettings.ENABLE_DRIVE && robotSettings.ENABLE_PNOOMATICS && robotSettings.ENABLE_BALL_SHIFTERS) {
            ballShifter = new DoubleSolenoid(robotSettings.PCM_ID, PneumaticsModuleType.CTREPCM, robotSettings.BALL_SHIFTERS_IN_ID, robotSettings.BALL_SHIFTERS_OUT_ID);
        }
        if (robotSettings.ENABLE_BUDDY_CLIMBER && robotSettings.ENABLE_PNOOMATICS) {
            buddyClimberLock = new DoubleSolenoid(robotSettings.PCM_ID, PneumaticsModuleType.CTREPCM, robotSettings.BUDDY_CLIMBER_LOCK_IN_ID, robotSettings.BUDDY_CLIMBER_LOCK_OUT_ID);
        }
    }

    @Override
    public SubsystemStatus getSubsystemStatus() {
        return SubsystemStatus.NOMINAL;
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
        return "Pneumatics";
    }
}
