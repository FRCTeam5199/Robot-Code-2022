package frc.controllers.basecontrollers;

import frc.robot.Robot;

/**
 * Some call it meme drive, i call it the only drive. Since the buildy bois are actively trying to screw over the
 * programmers, its time we take a shot back by giving them op controllers
 *
 * @see BaseController
 * @see DefaultControllerEnums.BopItButtons
 * @see DefaultControllerEnums.ButtonStatus
 */
public class BopItBasicController extends BaseController {

    BopItBasicController(Integer channel) {
        super(channel);
    }

    @Override
    public DefaultControllerEnums.ButtonStatus get(ControllerInterfaces.IDiscreteInput button) {
        if (button instanceof DefaultControllerEnums.BopItButtons || Robot.robotSettings.PERMIT_ROUGE_INPUT_MAPPING)
            return DefaultControllerEnums.ButtonStatus.get(controller.getRawButton(button.getChannel()));
        throw new IllegalArgumentException("Wrong mapping. Expected an enum of type " + DefaultControllerEnums.BopItButtons.class + " but got " + button.getClass().toString() + " instead");
    }
}
