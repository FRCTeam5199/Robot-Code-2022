package frc.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.misc.ISubsystem;

public interface ISensor extends ISubsystem {
    boolean isTriggered();
}
