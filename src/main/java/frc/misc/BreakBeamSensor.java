package frc.misc;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Robot.robotSettings;

public class BreakBeamSensor implements ISubsystem {
    DigitalInput indexerBreakBeam;
    DigitalInput intakeBreakBeam;
    boolean indexerIsBroken = false, intakeIsBroken = false;
    boolean isIndexerOperational = false, isIntakeOperational = false;

    public BreakBeamSensor() {
        addToMetaList();
        init();
    }

    public boolean getIndexerIsBroken() {
        return indexerIsBroken;
    }

    public boolean getIntakeIsBroken() {
        return intakeIsBroken;
    }

    @Override
    public void init() {
        indexerBreakBeam = new DigitalInput(robotSettings.INDEXER_BREAK_BEAM_ID);
        intakeBreakBeam = new DigitalInput(robotSettings.INTAKE_BREAK_BEAM_ID);
    }

    @Override
    public SubsystemStatus getSubsystemStatus() {
        return (isIndexerOperational && isIntakeOperational) ? SubsystemStatus.NOMINAL : SubsystemStatus.FAILED;
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
        if (indexerIsBroken == indexerBreakBeam.get())
            isIndexerOperational = true;
        if (intakeIsBroken == intakeBreakBeam.get())
            isIntakeOperational = true;
        indexerIsBroken = !indexerBreakBeam.get();
        intakeIsBroken = !intakeBreakBeam.get();
        if (robotSettings.DEBUG) {
            SmartDashboard.putBoolean("Intake Sensor Operational?", isIntakeOperational);
            SmartDashboard.putBoolean("Intake Broken", intakeIsBroken);
        }
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
        return "Break Beam Sensors";
    }
}
