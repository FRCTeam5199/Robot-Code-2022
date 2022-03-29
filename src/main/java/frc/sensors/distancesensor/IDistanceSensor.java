package frc.sensors.distancesensor;

import frc.ballstuff.intaking.Hopper;
import frc.ballstuff.intaking.Hopper2020;
import frc.misc.ISubsystem;
import frc.misc.SubsystemStatus;

/**
 * Used for optical distance sensors to detect distance from obstacles
 *
 * @see Hopper2020
 * @see Hopper
 * @see RevDistanceSensor
 */
public interface IDistanceSensor extends ISubsystem {
    @Override
    default SubsystemStatus getSubsystemStatus() {
        return getDistance() != -1 ? SubsystemStatus.NOMINAL : SubsystemStatus.FAILED;
    }

    /**
     * Gets the distance detected by the sensor
     *
     * @return the distance read by the sensor
     */
    double getDistance();

    @Override
    default String getSubsystemName() {
        return "Distance Sensor";
    }
}
