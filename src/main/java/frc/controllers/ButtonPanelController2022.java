package frc.controllers;

import frc.controllers.basecontrollers.BaseController;
import frc.controllers.basecontrollers.ControllerInterfaces;
import frc.controllers.basecontrollers.DefaultControllerEnums;

public class ButtonPanelController2022 extends BaseController {


    ButtonPanelController2022(Integer n) {
        super(n);
    }

    /**
     * Gets the Raw button value and returns true if it is pressed when it is run
     */
    @Override
    public DefaultControllerEnums.ButtonStatus get(ControllerInterfaces.IDiscreteInput button) {
        if (button instanceof ControllerEnums.ButtonPanelButtons2022)
            return DefaultControllerEnums.ButtonStatus.get(controller.getRawButton(button.getChannel()));
        throw new IllegalArgumentException("Wrong mapping. Expected an enum of type " + ControllerEnums.ButtonPanelButtons.class.toString() + " but got " + button.getClass().toString() + " instead");
    }


}
