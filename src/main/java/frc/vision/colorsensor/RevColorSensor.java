package frc.vision.colorsensor;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.misc.SubsystemStatus;

public class RevColorSensor extends ColorSensorV3 implements IColorSensor {
    /**
     * Constructs a ColorSensor.
     *
     * @param port The I2C port the color sensor is attached to
     */
    public RevColorSensor(I2C.Port port) {
        super(port);
        init();
    }

    public RevColorSensor() {
        super(I2C.Port.kOnboard);
    }

    @Override
    public void init() {
        addToMetaList();
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
    public SubsystemStatus getSubsystemStatus() {
        return SubsystemStatus.NOMINAL;
    }

    @Override
    public boolean isColor(Color checkAgainst) {
        return getColor().equals(checkAgainst);
    }

    @Override
    public String getSubsystemName() {
        return "Rev Color Sensor";
    }
}
