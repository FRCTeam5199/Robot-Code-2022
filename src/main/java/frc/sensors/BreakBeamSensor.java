package frc.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.misc.SubsystemStatus;

public class BreakBeamSensor implements ISensor {
    private final DigitalInput input;

    public BreakBeamSensor(int port) {
        input = new DigitalInput(port);
        init();
    }

    @Override
    public void init() {
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
        return "Break-Beam Sensors";
    }

    @Override
    public boolean isTriggered() {
        return !input.get();
    }
}
