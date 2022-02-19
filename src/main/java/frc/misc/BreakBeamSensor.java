package frc.misc;

import edu.wpi.first.wpilibj.DigitalInput;

public class BreakBeamSensor {
    DigitalInput breakBeam;
    public BreakBeamSensor (int index){
        breakBeam = new DigitalInput(index);
    }

    public boolean getBroken(){
        return !(breakBeam.get());
    }
}
