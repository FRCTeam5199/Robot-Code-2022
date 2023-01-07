package frc.controllers;

import frc.controllers.basecontrollers.*;

import java.util.function.Function;

import static frc.controllers.basecontrollers.BaseController.IValidController;

/**
 * These enums are used for each controller. If you make a new controller, create a new enum for its mappings here for
 * safekeeping and then implement a get function in {@link BaseController} and then create a new class extending
 * BaseController that overrides that get (the gets in BaseController should all throw exceptions so if an {@link
 * XBoxController xbox controller} is queried for a {@link DefaultControllerEnums.WiiAxis wii axis} it should throw a fit)
 *
 * @see BaseController
 */
public class ControllerEnums {
    public enum CustomControllers implements IValidController {
        BUTTON_PANEL_CONTROLLER(ButtonPanelController::new), BUTTTON_PANEL_CONTROLLER_2022(ButtonPanelController2022::new);

        private final Function<Integer, BaseController> constructor;

        CustomControllers(Function<Integer, BaseController> structor) {
            constructor = structor;
        }

        public Function<Integer, BaseController> getConstructor() {
            return constructor;
        }
    }

    /**
     * @see ButtonPanelController
     */
    public enum ButtonPanelButtons implements ControllerInterfaces.IDiscreteInput {
        RAISE_CLIMBER(1), LOWER_CLIMBER(2), CLIMBER_LOCK(3), CLIMBER_UNLOCK(4), BUDDY_CLIMB(5), AUX_TOP(6), AUX_BOTTOM(7), INTAKE_UP(8), INTAKE_DOWN(9), HOPPER_IN(10), HOPPER_OUT(11), TARGET(12), SOLID_SPEED(13);

        public final int AXIS_VALUE;

        ButtonPanelButtons(int value) {
            this.AXIS_VALUE = value;
        }

        @Override
        public int getChannel() {
            return AXIS_VALUE;
        }
    }

    /**
     * @see ButtonPanelController
     */
    public enum ButtonPanelTapedButtons implements ControllerInterfaces.IDiscreteInput {
        RAISE_CLIMBER(1), LOWER_CLIMBER(2), CLIMBER_LOCK(3), CLIMBER_UNLOCK(4), BUDDY_CLIMB(5), AUX_TOP(6), HOOD_POS_1(7), SINGLE_SHOT(8), HOOD_POS_2(9), SOLID_SPEED(10), HOOD_POS_3(11), TARGET(12), HOOD_POS_4(13);

        public final int AXIS_VALUE;

        ButtonPanelTapedButtons(int value) {
            this.AXIS_VALUE = value;
        }

        @Override
        public int getChannel() {
            return AXIS_VALUE;
        }
    }

    public enum ButtonPanelButtons2022 implements ControllerInterfaces.IDiscreteInput {
        LOW_SHOT(1), INTAKE_UP(6), INTAKE_DOWN(11), FENDER_SHOT(2), TARMAC_SHOT(7), FAR_SHOT(12), PIVOT_PISTON_UP(10), PIVOT_PISTON_DOWN(9), AUX_3(13), AUX_2(14), AUX_1(15), AUX_4(8), AUX_5(3), FIRST_STAGE_DOWN(4), FIRST_STAGE_UP(5);

        public final int AXIS_VALUE;

        ButtonPanelButtons2022(int value) {
            this.AXIS_VALUE = value;
        }

        @Override
        public int getChannel() {
            return AXIS_VALUE;
        }
    }
}