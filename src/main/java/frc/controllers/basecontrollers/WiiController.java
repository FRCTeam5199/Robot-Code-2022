package frc.controllers.basecontrollers;

import frc.robot.Robot;

/**
 * For mario kart drive. Google it if u dunno what it is Wii remote driver for windows:
 * https://www.julianloehr.de/educational-work/hid-wiimote/
 *
 * @see BaseController
 * @see DefaultControllerEnums.WiiAxis
 * @see DefaultControllerEnums.WiiButton
 * @see DefaultControllerEnums.ButtonStatus
 */
public class WiiController extends BaseController {
    WiiController(Integer n) {
        super(n);
    }

    /**
     * Gets the Raw axis value starting at 0
     *
     * @return the state of passed axis on a scale of [-1,1]
     * @see #get(ControllerInterfaces.IDiscreteInput)
     */
    @Override
    public double get(ControllerInterfaces.IContinuousInput axis) {
        if (axis instanceof DefaultControllerEnums.WiiAxis || Robot.robotSettings.PERMIT_ROUGE_INPUT_MAPPING)
            return controller.getRawAxis(axis.getChannel());
        throw new IllegalArgumentException("Wrong mapping. Expected an enum of type " + DefaultControllerEnums.WiiAxis.class + " but got " + axis.getClass().toString() + " instead");
    }

    /**
     * Gets the Raw button value and returns true if it is pressed when it is run
     *
     * @return the status of the passed button
     * @see #get(ControllerInterfaces.IContinuousInput)
     */
    @Override
    public DefaultControllerEnums.ButtonStatus get(ControllerInterfaces.IDiscreteInput button) {
        if (button instanceof DefaultControllerEnums.WiiButton || Robot.robotSettings.PERMIT_ROUGE_INPUT_MAPPING)
            return DefaultControllerEnums.ButtonStatus.get(controller.getRawButton(button.getChannel()));
        throw new IllegalArgumentException("Wrong mapping. Expected an enum of type " + DefaultControllerEnums.WiiButton.class + " but got " + button.getClass().toString() + " instead");
    }
}
