package frc.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.misc.SubsystemStatus;

public class LimitSwitchSensor implements ISensor {
    private final DigitalInput input;

    public LimitSwitchSensor(int port) {
        input = new DigitalInput(port);
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
        return "Limit Switch/Button";
    }

    @Override
    public boolean isTriggered() {
        return !input.get();
    }
}
