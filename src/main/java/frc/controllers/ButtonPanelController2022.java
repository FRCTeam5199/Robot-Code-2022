package frc.controllers;

import frc.robot.Robot;
import frc.controllers.ControllerEnums.ButtonPanelButtons;

public class ButtonPanelController2022 extends BaseController {


    ButtonPanelController2022(Integer n) {
        super(n);
    }

    /**
     * Gets the Raw button value and returns true if it is pressed when it is run
     */
    @Override
    public ControllerEnums.ButtonStatus get(ControllerInterfaces.IDiscreteInput button) {
        if (button instanceof ControllerEnums.ButtonPanelButtons2022)
            return ControllerEnums.ButtonStatus.get(controller.getRawButton(button.getChannel()));
        throw new IllegalArgumentException("Wrong mapping. Expected an enum of type " + ControllerEnums.ButtonPanelButtons.class.toString() + " but got " + button.getClass().toString() + " instead");
    }


}
