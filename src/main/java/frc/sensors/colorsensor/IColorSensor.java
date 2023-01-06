package frc.sensors.colorsensor;

import edu.wpi.first.wpilibj.util.Color;
import frc.misc.ISubsystem;
import frc.misc.SubsystemStatus;

/**
 * Used forr color sensors to detect color from objects
 *
 * @see RevColorSensor
 */
public interface IColorSensor extends ISubsystem {
    @Override
    default SubsystemStatus getSubsystemStatus() {
        return SubsystemStatus.NOMINAL; //getColor() != C ? SubsystemStatus.NOMINAL : SubsystemStatus.FAILED;
    }

    /**
     * Gets the color detected by the sensor
     *
     * @return the color read by the sensor
     */
    Color getColor();

    boolean isColor(Color checkAgainst);

    @Override
    default String getSubsystemName() {
        return "Color Sensor";
    }
}
