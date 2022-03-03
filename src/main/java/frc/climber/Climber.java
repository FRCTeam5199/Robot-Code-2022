package frc.climber;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.controllers.BaseController;
import frc.controllers.ControllerEnums;
import frc.controllers.ControllerEnums.ButtonStatus;
import frc.misc.ISubsystem;
import frc.misc.InitializationFailureException;
import frc.misc.SubsystemStatus;
import frc.motors.AbstractMotorController;
import frc.motors.SparkMotorController;
import frc.motors.TalonMotorController;
import frc.motors.VictorMotorController;
import frc.robot.Robot;

import java.util.Objects;

import static frc.controllers.ControllerEnums.ButtonPanelButtons.*;
import static frc.robot.Robot.robotSettings;


/**
 * Allows the robot to climb to the top of the bar. We're going to the moon!
 *
 * @author Smaltin
 */
public class Climber implements ISubsystem {
    public BaseController joystick, buttonpanel;
    private AbstractMotorController[] climberMotors;
    private AbstractMotorController climberStg1, climberStg2;
    private boolean isLocked = false;

    public Climber() {
        addToMetaList();
        init();
    }

    @Override
    public void init() {
        if (robotSettings.CLIMBER_CONTROL_STYLE != ClimberControlStyles.OLD_STANDARD_2022 && robotSettings.CLIMBER_CONTROL_STYLE != ClimberControlStyles.STANDARD_2022)
            createMotors();
        else
            createCoolerMotors();
        createControllers();
    }

    @Override
    public SubsystemStatus getSubsystemStatus() {
        return SubsystemStatus.NOMINAL;
    }

    @Override
    public void updateTest() {

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
        switch (robotSettings.CLIMBER_CONTROL_STYLE) {
            case STANDARD: {
                if (buttonpanel.get(LOWER_CLIMBER) == ButtonStatus.DOWN) {
                    for (AbstractMotorController motor : climberMotors) {
                        motor.moveAtPercent(-0.8);
                    }
                } else if (buttonpanel.get(RAISE_CLIMBER) == ButtonStatus.DOWN && !isLocked) {
                    for (AbstractMotorController motor : climberMotors) {
                        motor.moveAtPercent(0.8);
                    }
                } else {
                    for (AbstractMotorController motor : climberMotors) {
                        motor.moveAtPercent(0);
                    }
                }
                if (buttonpanel.get(CLIMBER_LOCK) == ButtonStatus.DOWN) {
                    climberLocks(true);
                } else if (buttonpanel.get(CLIMBER_UNLOCK) == ButtonStatus.DOWN) {
                    climberLocks(false);
                }
            }
            break;
            case STANDARD_2022: {
                if (buttonpanel.get(ControllerEnums.ButtonPanelButtons2022.FIRST_STAGE_UP) == ButtonStatus.DOWN && !isLocked) {
                    climberStg1.moveAtPercent(-0.8);
                } else if (buttonpanel.get(ControllerEnums.ButtonPanelButtons2022.FIRST_STAGE_DOWN) == ButtonStatus.DOWN && !isLocked) {
                    climberStg1.moveAtPercent(0.8);
                } else {
                    climberStg1.moveAtPercent(0);
                }
                /*if (buttonpanel.get(RAISE_CLIMBER) == ButtonStatus.DOWN) {
                    climberStg2.moveAtPercent(0.8);
                } else */
                if (buttonpanel.get(ControllerEnums.ButtonPanelButtons2022.SECOND_STAGE_CLIMB) == ButtonStatus.DOWN) {
                    climberStg2.moveAtPercent(-0.8);
                    climberLocks(true);
                } else if (buttonpanel.get(ControllerEnums.ButtonPanelButtons2022.AUX_3) == ButtonStatus.DOWN) {
                    climberStg2.moveAtPercent(0.8);
                } else {
                    climberStg2.moveAtPercent(0);
                }

                if (buttonpanel.get(ControllerEnums.ButtonPanelButtons2022.AUX_2) == ButtonStatus.DOWN) {
                    climberLocks(true);
                } else if (buttonpanel.get(ControllerEnums.ButtonPanelButtons2022.AUX_1) == ButtonStatus.DOWN) {
                    climberLocks(false);
                }
            }
            break;
            case OLD_STANDARD_2022: {
                if (buttonpanel.get(AUX_TOP) == ButtonStatus.DOWN) {
                    climberStg1.moveAtPercent(0.8);
                } else if (buttonpanel.get(AUX_BOTTOM) == ButtonStatus.DOWN) {
                    climberStg1.moveAtPercent(-0.8);
                } else {
                    climberStg1.moveAtPercent(0);
                }

                if (buttonpanel.get(RAISE_CLIMBER) == ButtonStatus.DOWN) {
                    //climberStg2.moveAtPercent(0.8);
                } else if (buttonpanel.get(LOWER_CLIMBER) == ButtonStatus.DOWN) {
                    climberStg2.moveAtPercent(-0.8);
                } else {
                    climberStg2.moveAtPercent(0);
                }

                if (buttonpanel.get(CLIMBER_LOCK) == ButtonStatus.DOWN) {
                    climberLocks(true);
                } else if (buttonpanel.get(CLIMBER_UNLOCK) == ButtonStatus.DOWN) {
                    climberLocks(false);
                }
            }
            break;
            default: {
                throw new IllegalStateException("There is no UI configuration for " + robotSettings.CLIMBER_CONTROL_STYLE.name() + " to control the climber. Please implement me");
            }
        }
        //if (robotSettings.CLIMBER_CONTROL_STYLE == ClimberControlStyles.STANDARD || robotSettings.CLIMBER_CONTROL_STYLE == ClimberControlStyles.STANDARD_2022) {
    }

    @Override
    public void initTest() {
        climberStg1.setBrake(false);
        climberStg2.setBrake(false);
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
    }

    @Override
    public void initGeneric() {
        climberStg1.setBrake(true);
        climberStg2.setBrake(true);
    }

    @Override
    public String getSubsystemName() {
        return "Climber";
    }

    public void climberLocks(boolean deployed) {
        if (robotSettings.ENABLE_PNOOMATICS)
            Robot.pneumatics.climberLock.set(deployed ? Value.kForward : Value.kReverse);
        isLocked = deployed;
    }

    private void createCoolerMotors() {
        double s2rfstg1 = 1;
        switch (robotSettings.CLIMBER_MOTOR_TYPE) {
            case TALON_FX: {
                climberStg1 = new TalonMotorController(robotSettings.CLIMBER_STG1_MOTOR_ID);
                s2rfstg1 = 600 / robotSettings.CTRE_SENSOR_UNITS_PER_ROTATION;
            }
            break;
            case VICTOR: {
                climberStg1 = new VictorMotorController(robotSettings.CLIMBER_STG1_MOTOR_ID);
                s2rfstg1 = 600 / robotSettings.CTRE_SENSOR_UNITS_PER_ROTATION;
            }
            break;
            case CAN_SPARK_MAX: {
                climberStg1 = new SparkMotorController(robotSettings.CLIMBER_STG1_MOTOR_ID);
            }
            break;
            default:
                throw new IllegalStateException("No such supported climber motor config for " + robotSettings.CLIMBER_MOTOR_TYPE.name());
        }
        climberStg1.setSensorToRealDistanceFactor(s2rfstg1);

        double s2rfstg2 = 1;
        switch (robotSettings.CLIMBER_STG2_MOTOR_TYPE) {
            case TALON_FX: {
                climberStg2 = new TalonMotorController(robotSettings.CLIMBER_STG2_MOTOR_ID);
                s2rfstg2 = 600 / robotSettings.CTRE_SENSOR_UNITS_PER_ROTATION;
            }
            break;
            case VICTOR: {
                climberStg2 = new VictorMotorController(robotSettings.CLIMBER_STG2_MOTOR_ID);
                s2rfstg2 = 600 / robotSettings.CTRE_SENSOR_UNITS_PER_ROTATION;
            }
            break;
            case CAN_SPARK_MAX: {
                climberStg2 = new SparkMotorController(robotSettings.CLIMBER_STG2_MOTOR_ID);
            }
            break;
            default:
                throw new IllegalStateException("No such supported stage 2 climber motor config for " + robotSettings.CLIMBER_MOTOR_TYPE.name());
        }
        climberStg2.setSensorToRealDistanceFactor(s2rfstg2);
    }

    private void createMotors() {
        climberMotors = new AbstractMotorController[robotSettings.CLIMBER_MOTOR_IDS.length];
        for (int indexer = 0; indexer < robotSettings.CLIMBER_MOTOR_IDS.length; indexer++) {
            switch (Robot.robotSettings.CLIMBER_MOTOR_TYPE) {
                case VICTOR:
                    climberMotors[indexer] = new VictorMotorController(robotSettings.CLIMBER_MOTOR_IDS[indexer]);
                    break;
                case TALON_FX:
                    climberMotors[indexer] = new TalonMotorController(robotSettings.CLIMBER_MOTOR_IDS[indexer]);
                    break;
                case CAN_SPARK_MAX:
                    climberMotors[indexer] = new SparkMotorController(robotSettings.CLIMBER_MOTOR_IDS[indexer]);
                    break;
                default:
                    throw new InitializationFailureException("DriveManager does not have a suitible constructor for " + robotSettings.CLIMBER_MOTOR_TYPE.name(), "Add an implementation in the init for climber");
            }
            if (indexer % 2 != 0) {
                climberMotors[indexer].setInverted(true);
            }
        }
    }

    private void createControllers() {
        switch (robotSettings.CLIMBER_CONTROL_STYLE) {
            case FLIGHT_STICK:
                joystick = BaseController.createOrGet(robotSettings.FLIGHT_STICK_USB_SLOT, BaseController.Controllers.JOYSTICK_CONTROLLER);
                break;
            case STANDARD_2022:
                buttonpanel = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT, BaseController.Controllers.BUTTTON_PANEL_CONTROLLER_2022);
                break;
            case OLD_STANDARD_2022:
            case STANDARD:
                buttonpanel = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT, BaseController.Controllers.BUTTON_PANEL_CONTROLLER);
                break;
            case XBOX_CONTROLLER:
                joystick = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT, BaseController.Controllers.XBOX_CONTROLLER);
                break;
            case BOP_IT:
                joystick = BaseController.createOrGet(3, BaseController.Controllers.BOP_IT_CONTROLLER);
                break;
            case DRUM_TIME:
                joystick = BaseController.createOrGet(5, BaseController.Controllers.DRUM_CONTROLLER);
                break;
            case WII:
                joystick = BaseController.createOrGet(4, BaseController.Controllers.WII_CONTROLLER);
                break;
            case GUITAR:
                joystick = BaseController.createOrGet(6, BaseController.Controllers.SIX_BUTTON_GUITAR_CONTROLLER);
                break;
            default:
                throw new IllegalStateException("There is no UI configuration for " + robotSettings.INTAKE_CONTROL_STYLE.name() + " to control the shooter. Please implement me");
        }
    }

    public enum ClimberControlStyles {
        STANDARD,
        OLD_STANDARD_2022,
        STANDARD_2022,
        WII,
        DRUM_TIME,
        GUITAR,
        BOP_IT,
        FLIGHT_STICK,
        XBOX_CONTROLLER;


        private static SendableChooser<Climber.ClimberControlStyles> myChooser;

        public static SendableChooser<Climber.ClimberControlStyles> getSendableChooser() {
            return Objects.requireNonNullElseGet(myChooser, () -> {
                myChooser = new SendableChooser<>();
                for (Climber.ClimberControlStyles style : Climber.ClimberControlStyles.values())
                    myChooser.addOption(style.name(), style);
                return myChooser;
            });
        }
    }
}