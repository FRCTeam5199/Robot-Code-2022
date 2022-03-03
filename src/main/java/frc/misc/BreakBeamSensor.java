package frc.misc;

import edu.wpi.first.wpilibj.DigitalInput;

import static frc.robot.Robot.robotSettings;

public class BreakBeamSensor implements ISubsystem {
    DigitalInput breakBeam;
    boolean isBroken = false;

    public BreakBeamSensor() {
        addToMetaList();
        init();
    }

    public boolean getBroken() {
        return isBroken;
    }

    @Override
    public void init() {
        
        breakBeam = new DigitalInput(robotSettings.BREAK_BEAM_ID);
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
        isBroken = !breakBeam.get();
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
        return "Break Beam Sensor";
    }
}
