package frc.controllers.basecontrollers;

import frc.robot.Robot;

/**
 * Who doesn't like playing guitar hero while also driving a robot, am I right? Now I can jam out while slowly and
 * methodically destroying the robot.
 *
 * @author Smaltin
 * @see BaseController
 * @see DefaultControllerEnums.SixKeyGuitarButtons
 * @see DefaultControllerEnums.SixKeyGuitarAxis
 * @see DefaultControllerEnums.ButtonStatus
 */
public class SixButtonGuitarController extends BaseController {
    SixButtonGuitarController(Integer channel) {
        super(channel);
    }

    @Override
    public DefaultControllerEnums.ButtonStatus get(ControllerInterfaces.IDiscreteInput button) {
        if (button instanceof DefaultControllerEnums.SixKeyGuitarButtons || Robot.robotSettings.PERMIT_ROUGE_INPUT_MAPPING)
            return DefaultControllerEnums.ButtonStatus.get(controller.getRawButton(button.getChannel()));
        throw new IllegalArgumentException("Wrong mapping. Expected an enum of type " + DefaultControllerEnums.SixKeyGuitarButtons.class + " but got " + button.getClass().toString() + " instead");
    }

    public double get(ControllerInterfaces.IContinuousInput axis) {
        if (axis instanceof DefaultControllerEnums.SixKeyGuitarAxis || Robot.robotSettings.PERMIT_ROUGE_INPUT_MAPPING)
            return controller.getRawAxis(axis.getChannel());
        throw new IllegalArgumentException("Wrong mapping. Expected an enum of type " + DefaultControllerEnums.SixKeyGuitarButtons.class + " but got " + axis.getClass().toString() + " instead");
    }
}
