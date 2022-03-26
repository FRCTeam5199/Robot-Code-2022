package frc.misc;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.*;

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
    public DoubleSolenoid hoodArticulator;
    public DoubleSolenoid indexerBlocker;
    public DoubleSolenoid climberPiston;
    public Compressor compressor;
    public PneumaticHub pneumaticsHub;
    private final NetworkTableEntry
            compressorOverride = UserInterface.COMPRESSOR_TOGGLE.getEntry(),
            compressorToggle = UserInterface.COMPRESSOR_STATE.getEntry();

    public Pneumatics() {
        addToMetaList();
        init();
    }

    @Override
    public void init() {
        if (robotSettings.PNEUMATICS_MODULE_TYPE == PneumaticsModuleType.REVPH) {
            pneumaticsHub = new PneumaticHub(robotSettings.PCM_ID);
            pneumaticsHub.clearStickyFaults();
        } else {
            compressor = new Compressor(robotSettings.PNEUMATICS_MODULE_TYPE);
        }
        if (robotSettings.ENABLE_INTAKE && robotSettings.ENABLE_PNOOMATICS) {
            solenoidIntake = new DoubleSolenoid(robotSettings.PCM_ID, robotSettings.PNEUMATICS_MODULE_TYPE, robotSettings.INTAKE_OUT_ID, robotSettings.INTAKE_IN_ID);
        }
        if (robotSettings.ENABLE_CLIMBER && robotSettings.ENABLE_PNOOMATICS && robotSettings.ENABLE_CLIMBER_LOCK) {
            climberLock = new DoubleSolenoid(robotSettings.PCM_ID, robotSettings.PNEUMATICS_MODULE_TYPE, robotSettings.CLIMBER_LOCK_IN_ID, robotSettings.CLIMBER_LOCK_OUT_ID);
        }
        if (robotSettings.ENABLE_SHOOTER && robotSettings.ENABLE_PNOOMATICS && robotSettings.ENABLE_SHOOTER_COOLING) {
            shooterCooling = new Solenoid(robotSettings.PCM_ID, robotSettings.PNEUMATICS_MODULE_TYPE, robotSettings.SHOOTER_COOLING_ID);
        }
        if (robotSettings.ENABLE_DRIVE && robotSettings.ENABLE_PNOOMATICS && robotSettings.ENABLE_BALL_SHIFTERS) {
            ballShifter = new DoubleSolenoid(robotSettings.PCM_ID, robotSettings.PNEUMATICS_MODULE_TYPE, robotSettings.BALL_SHIFTERS_IN_ID, robotSettings.BALL_SHIFTERS_OUT_ID);
        }
        if (robotSettings.ENABLE_BUDDY_CLIMBER && robotSettings.ENABLE_PNOOMATICS) {
            buddyClimberLock = new DoubleSolenoid(robotSettings.PCM_ID, robotSettings.PNEUMATICS_MODULE_TYPE, robotSettings.BUDDY_CLIMBER_LOCK_IN_ID, robotSettings.BUDDY_CLIMBER_LOCK_OUT_ID);
        }
        if (robotSettings.ENABLE_HOOD_PISTON && robotSettings.ENABLE_PNOOMATICS) {
            hoodArticulator = new DoubleSolenoid(robotSettings.PCM_ID, robotSettings.PNEUMATICS_MODULE_TYPE, robotSettings.HOOD_ARTICULATOR_IN_ID, robotSettings.HOOD_ARTICULATOR_OUT_ID);
        }
        if (robotSettings.ENABLE_INDEXER && robotSettings.ENABLE_HOPPER && robotSettings.ENABLE_PNOOMATICS && robotSettings.ENABLE_INDEXER_PISTON_BLOCK) {
            indexerBlocker = new DoubleSolenoid(robotSettings.PCM_ID, robotSettings.PNEUMATICS_MODULE_TYPE, robotSettings.INDEXER_BLOCK_IN_ID, robotSettings.INDEXER_BLOCK_OUT_ID);
        }
        if (robotSettings.ENABLE_CLIMBER && robotSettings.ENABLE_CLIMBER_PISTON) {
            climberPiston = new DoubleSolenoid(robotSettings.PCM_ID, robotSettings.PNEUMATICS_MODULE_TYPE, robotSettings.CLIMBER_PISTON_IN_ID, robotSettings.CLIMBER_PISTON_OUT_ID);
        }
    }

    @Override
    public SubsystemStatus getSubsystemStatus() {
        return SubsystemStatus.NOMINAL;
    }

    @Override
    public void updateTest() {
        updateGeneric();
    }

    @Override
    public void updateTeleop() {
        updateGeneric();
    }

    @Override
    public void updateAuton() {
        updateGeneric();
    }

    @Override
    public void updateGeneric() {
        if (robotSettings.PNEUMATICS_MODULE_TYPE == PneumaticsModuleType.CTREPCM) {
            if (compressorOverride.getBoolean(false)) {
                System.out.println("Overriding compressor to " + compressorToggle.getBoolean(true));
                if (compressorToggle.getBoolean(true)) {
                    compressor.enableDigital();
                } else {
                    compressor.disable();
                }
            } else {
                compressor.enableDigital();
            }
        } else if (robotSettings.PNEUMATICS_MODULE_TYPE == PneumaticsModuleType.REVPH) {
            if (compressorOverride.getBoolean(false)) {
                if (compressorToggle.getBoolean(true)) {
                    pneumaticsHub.enableCompressorDigital();
                } else {
                    pneumaticsHub.disableCompressor();
                }
            } else {
                pneumaticsHub.enableCompressorDigital();
            }
        }
    }

    @Override
    public void initTest() {

    }

    @Override
    public void initTeleop() {
        initGeneric();
    }

    @Override
    public void initAuton() {
        initGeneric();
    }

    @Override
    public void initDisabled() {
        if (robotSettings.ENABLE_INDEXER && robotSettings.ENABLE_HOPPER && robotSettings.ENABLE_PNOOMATICS && robotSettings.ENABLE_INDEXER_PISTON_BLOCK)
            indexerBlocker.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void initGeneric() {
        if (robotSettings.ENABLE_INDEXER && robotSettings.ENABLE_HOPPER && robotSettings.ENABLE_PNOOMATICS && robotSettings.ENABLE_INDEXER_PISTON_BLOCK)
            indexerBlocker.set(DoubleSolenoid.Value.kForward);
        if (robotSettings.ENABLE_CLIMBER && robotSettings.ENABLE_PNOOMATICS && robotSettings.ENABLE_CLIMBER_LOCK)
            climberLock.set(DoubleSolenoid.Value.kReverse);
        if (robotSettings.ENABLE_HOOD_PISTON && robotSettings.ENABLE_PNOOMATICS)
            hoodArticulator.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public String getSubsystemName() {
        return "Pneumatics";
    }
}
