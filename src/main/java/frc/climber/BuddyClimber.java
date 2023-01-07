package frc.climber;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.controllers.basecontrollers.BaseController;
import frc.controllers.ControllerEnums;
import frc.controllers.basecontrollers.DefaultControllerEnums;
import frc.misc.ISubsystem;
import frc.misc.SubsystemStatus;
import frc.robot.Robot;

import static frc.robot.Robot.robotSettings;

public class BuddyClimber implements ISubsystem {
    public BaseController joystick, buttonpanel;
    private boolean buddyLock = false;

    public BuddyClimber() {
        addToMetaList();
        init();
    }

    @Override
    public void init() {
        createControllers();
    }

    @Override
    public SubsystemStatus getSubsystemStatus() {
        return SubsystemStatus.NOMINAL;
    }

    @Override
    public void updateTest() {
        updateGeneric();
    }

    @Override
    public void updateTeleop() {
        updateGeneric();
    }

    @Override
    public void updateAuton() {

    }

    @Override
    public void updateGeneric() {
        if (robotSettings.CLIMBER_CONTROL_STYLE == Climber.ClimberControlStyles.STANDARD) {
            if (buttonpanel.get(ControllerEnums.ButtonPanelButtons.BUDDY_CLIMB) == DefaultControllerEnums.ButtonStatus.DOWN)
                buddyLock = false;
        } else {
            throw new IllegalStateException("There is no UI configuration for " + robotSettings.CLIMBER_CONTROL_STYLE.name() + " to control the climber. Please implement me");
        }
        buddyClimberLocks(buddyLock);
    }

    @Override
    public void initTest() {
        initGeneric();
    }

    @Override
    public void initTeleop() {
        initGeneric();
    }

    @Override
    public void initAuton() {
        initGeneric();
    }

    @Override
    public void initDisabled() {
        buddyClimberLocks(true);
    }

    @Override
    public void initGeneric() {
        buddyLock = true;
    }

    @Override
    public String getSubsystemName() {
        return "Buddy Climber";
    }

    public void buddyClimberLocks(boolean deployed) {
        if (robotSettings.ENABLE_PNOOMATICS)
            Robot.pneumatics.buddyClimberLock.set(deployed ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
    }

    private void createControllers() {
        switch (robotSettings.INTAKE_CONTROL_STYLE) {
            case FLIGHT_STICK:
                joystick = BaseController.createOrGet(robotSettings.FLIGHT_STICK_USB_SLOT, BaseController.DefaultControllers.JOYSTICK_CONTROLLER);
            case STANDARD:
                buttonpanel = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT, ControllerEnums.CustomControllers.BUTTON_PANEL_CONTROLLER);
                break;
            case XBOX_CONTROLLER:
                joystick = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT, BaseController.DefaultControllers.XBOX_CONTROLLER);
            case BOP_IT:
                joystick = BaseController.createOrGet(3, BaseController.DefaultControllers.BOP_IT_CONTROLLER);
                break;
            case DRUM_TIME:
                joystick = BaseController.createOrGet(5, BaseController.DefaultControllers.DRUM_CONTROLLER);
                break;
            case WII:
                joystick = BaseController.createOrGet(4, BaseController.DefaultControllers.WII_CONTROLLER);
                break;
            case GUITAR:
                joystick = BaseController.createOrGet(6, BaseController.DefaultControllers.SIX_BUTTON_GUITAR_CONTROLLER);
                break;
            default:
                throw new IllegalStateException("There is no UI configuration for " + robotSettings.INTAKE_CONTROL_STYLE.name() + " to control the shooter. Please implement me");
        }
    }
}
