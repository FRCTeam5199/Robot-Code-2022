package frc.controllers.basecontrollers;

import frc.robot.Robot;

/**
 * Some call it meme drive, i call it the only drive. Since the buildy bois are actively trying to screw over the
 * programmers, its time we take a shot back by giving them op controllers
 *
 * @author jojo2357
 * @see BaseController
 * @see DefaultControllerEnums.DrumButton
 * @see DefaultControllerEnums.Drums
 * @see DefaultControllerEnums.ButtonStatus
 */
public class DrumTimeController extends BaseController {
    DrumTimeController(Integer channel) {
        super(channel);
    }

    @Override
    public DefaultControllerEnums.ButtonStatus get(ControllerInterfaces.IDiscreteInput button) {
        if (button instanceof DefaultControllerEnums.DrumButton)
            return DefaultControllerEnums.ButtonStatus.get(controller.getRawButton(button.getChannel()));
        else if (button instanceof DefaultControllerEnums.Drums || Robot.robotSettings.PERMIT_ROUGE_INPUT_MAPPING)
            return DefaultControllerEnums.ButtonStatus.get(controller.getRawButton(8) && controller.getRawButton(button.getChannel()));
        throw new IllegalArgumentException("Wrong mapping. Expected an enum of type " + DefaultControllerEnums.DrumButton.class + " but got " + button.getClass().toString() + " instead");
    }
}
