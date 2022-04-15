package frc.motors.followers;

import frc.motors.AbstractMotorController;
import frc.motors.TalonMotorController;

/**
 * This works to wrap Falcon500's and maybe some other motors
 */
public class TalonFollowerMotorController extends AbstractFollowerMotorController {
    public TalonFollowerMotorController(String bus, int... ids) {
        motors = new TalonMotorController[ids.length];
        for (int i = 0; i < ids.length; i++)
            motors[i] = new TalonMotorController(bus, ids[i]);
    }

    @Override
    public void invert(boolean invert) {
        for (AbstractMotorController motor : motors)
            motor.setInverted(invert);
    }
}
