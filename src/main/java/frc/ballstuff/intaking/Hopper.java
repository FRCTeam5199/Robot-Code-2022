package frc.ballstuff.intaking;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.ballstuff.shooting.Shooter;
import frc.controllers.BaseController;
import frc.controllers.ControllerEnums;
import frc.misc.ISubsystem;
import frc.misc.SubsystemStatus;
import frc.misc.UserInterface;
import frc.motors.AbstractMotorController;
import frc.motors.SparkMotorController;
import frc.motors.TalonMotorController;
import frc.motors.VictorMotorController;
import frc.selfdiagnostics.MotorDisconnectedIssue;
import frc.vision.distancesensor.IDistanceSensor;
import frc.vision.distancesensor.RevDistanceSensor;

import java.util.Objects;

import static com.revrobotics.Rev2mDistanceSensor.Port.kOnboard;
import static com.revrobotics.Rev2mDistanceSensor.RangeProfile.kHighAccuracy;
import static com.revrobotics.Rev2mDistanceSensor.Unit.kInches;
import static frc.robot.Robot.breakBeam;
import static frc.robot.Robot.robotSettings;

/**
 * The Hopper subsystem effectively takes a ball from the front (where the {@link Intake intake}
 * is) to the {@link frc.ballstuff.shooting.Shooter shooter}.
 */
public class Hopper implements ISubsystem {
    private static final boolean DEBUG = false;
    public AbstractMotorController agitator, agitatorTop, indexer;
    public IDistanceSensor indexSensor;
    public boolean agitatorActive = false, indexerActive = false, agitatorTopbarActive = false;
    private BaseController controller, panel;

    public Hopper() {
        addToMetaList();
        init();
    }

    /**
     * Creates xbox controller n stuff
     *
     * @throws UnsupportedOperationException when there is no configuration for {@link frc.robot.robotconfigs.DefaultConfig#DRIVE_STYLE}
     */
    private void initMisc() throws UnsupportedOperationException {
        switch (robotSettings.HOPPER_CONTROL_STYLE) {
            case STANDARD:
                break;
            case STANDARD_2022:
                panel = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT, BaseController.Controllers.BUTTON_PANEL_CONTROLLER);
                break;
            case PRACTICE_2022:
                controller = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT, BaseController.Controllers.XBOX_CONTROLLER);
                break;
            default:
                throw new UnsupportedOperationException("There is no UI configuration for " + robotSettings.DRIVE_STYLE.name() + " to control the drivetrain. Please implement me");
        }
        if (robotSettings.DEBUG && DEBUG && controller != null) System.out.println("Created a " + controller);
    }

    @Override
    public void init() {
        if (robotSettings.ENABLE_INDEXER_AUTO_INDEX && !robotSettings.ENABLE_BREAK_BEAM) {
            indexSensor = new RevDistanceSensor(kOnboard, kInches, kHighAccuracy);
            System.out.println("Enabling index sensor.");
        }
        createAndInitMotors();
        initMisc();
    }


    private void createAndInitMotors() throws IllegalStateException {
        if (robotSettings.ENABLE_AGITATOR) switch (robotSettings.AGITATOR_MOTOR_TYPE) {
            case CAN_SPARK_MAX:
                agitator = new SparkMotorController(robotSettings.AGITATOR_MOTOR_ID);
                agitator.setSensorToRealDistanceFactor(1);
                break;
            case TALON_FX:
                agitator = new TalonMotorController(robotSettings.AGITATOR_MOTOR_ID);
                agitator.setSensorToRealDistanceFactor(600 / robotSettings.CTRE_SENSOR_UNITS_PER_ROTATION);
                break;
            case VICTOR:
                agitator = new VictorMotorController(robotSettings.AGITATOR_MOTOR_ID);
                agitator.setSensorToRealDistanceFactor(600 / robotSettings.CTRE_SENSOR_UNITS_PER_ROTATION);
                break;
            default:
                throw new IllegalStateException("No such supported hopper agitator motor config for " + robotSettings.AGITATOR_MOTOR_TYPE.name());
        }
        if (robotSettings.ENABLE_AGITATOR_TOP) {
            switch (robotSettings.AGITATOR_MOTOR_TYPE) {
                case CAN_SPARK_MAX:
                    agitatorTop = new SparkMotorController(robotSettings.AGITATOR_TOPBAR_MOTOR_ID);
                    agitatorTop.setSensorToRealDistanceFactor(1);
                case TALON_FX:
                    agitatorTop = new TalonMotorController(robotSettings.AGITATOR_TOPBAR_MOTOR_ID);
                    agitatorTop.setSensorToRealDistanceFactor(600 / robotSettings.CTRE_SENSOR_UNITS_PER_ROTATION);
                    break;
                case VICTOR:
                    agitatorTop = new VictorMotorController(robotSettings.AGITATOR_TOPBAR_MOTOR_ID);
                    agitatorTop.setSensorToRealDistanceFactor(600 / robotSettings.CTRE_SENSOR_UNITS_PER_ROTATION);
                    break;
                default:
                    throw new IllegalStateException("No such supported hopper agitator topbar motor config for " + robotSettings.AGITATOR_TOP_MOTOR_TYPE.name());
            }
            agitatorTop.setInverted(true);
        }
        if (robotSettings.ENABLE_INDEXER) switch (robotSettings.INDEXER_MOTOR_TYPE) {
            case CAN_SPARK_MAX:
                indexer = new SparkMotorController(robotSettings.INDEXER_MOTOR_ID);
                indexer.setSensorToRealDistanceFactor(1);
                break;
            case TALON_FX:
                indexer = new TalonMotorController(robotSettings.INDEXER_MOTOR_ID);
                indexer.setSensorToRealDistanceFactor(600 / robotSettings.CTRE_SENSOR_UNITS_PER_ROTATION);
                break;
            case VICTOR:
                indexer = new VictorMotorController(robotSettings.INDEXER_MOTOR_ID);
                indexer.setSensorToRealDistanceFactor(600 / robotSettings.CTRE_SENSOR_UNITS_PER_ROTATION);
                break;
            default:
                throw new IllegalStateException("No such supported hopper indexer motor config for " + robotSettings.INDEXER_MOTOR_TYPE.name());
        }
    }

    @Override
    public SubsystemStatus getSubsystemStatus() {
        return !indexer.isFailed() && !agitator.isFailed() ? SubsystemStatus.NOMINAL : SubsystemStatus.FAILED;
    }

    @Override
    public void updateTest() {
        if (DEBUG && robotSettings.DEBUG) {
            UserInterface.smartDashboardPutNumber("indexer sensor", indexerSensorRange());
            UserInterface.smartDashboardPutBoolean("isIndexed", isIndexed());
        }
    }

    @Override
    public void updateTeleop() {
        updateGeneric();
    }

    @Override
    public void updateAuton() {
        if (!robotSettings.autonComplete) {
            updateTeleop();
            //agitator.moveAtPercent(0.6);
        } else {
            if (robotSettings.ENABLE_AGITATOR) agitator.moveAtPercent(0);
            if (robotSettings.ENABLE_INDEXER) indexer.moveAtPercent(0);
            if (robotSettings.ENABLE_AGITATOR_TOP) agitatorTop.moveAtPercent(0);
        }
    }

    /**
     * Uses the distance sensor to determine if there is a ball in the indxer. Enable and disable the indexer using
     * {@link frc.robot.robotconfigs.DefaultConfig#ENABLE_INDEXER_AUTO_INDEX}
     *
     * @return distance as read by {@link #indexSensor} assuming it is {@link frc.robot.robotconfigs.DefaultConfig#ENABLE_INDEXER_AUTO_INDEX
     * enabled}
     */
    public double indexerSensorRange() {
        if (robotSettings.ENABLE_INDEXER_AUTO_INDEX) {
            if (!robotSettings.ENABLE_BREAK_BEAM) {
                return indexSensor.getDistance();
            } else {
                return -3;
            }
        }
        return -2;
    }

    public boolean isIndexed() {
        if (robotSettings.ENABLE_BREAK_BEAM) {
            return breakBeam.getBroken();
        } else {
            return robotSettings.ENABLE_INDEXER_AUTO_INDEX && indexerSensorRange() < robotSettings.INDEXER_DETECTION_CUTOFF_DISTANCE && indexerSensorRange() > 0;
        }
    }

    /**
     * Runs every tick. Runs the indexer and agitator motors.
     */
    @Override
    public void updateGeneric() {
        MotorDisconnectedIssue.handleIssue(this, agitator, indexer);
        switch (robotSettings.HOPPER_CONTROL_STYLE) {
            case STANDARD: {
                if (!indexerActive && !agitatorActive) {
                    if (robotSettings.ENABLE_INDEXER) {
                        if (robotSettings.ENABLE_INDEXER_AUTO_INDEX) {
                            indexer.moveAtPercent(!isIndexed() ? 0.3 : 0);
                        } else {
                            indexer.moveAtPercent(0);
                        }
                    } //2021 COMP 4 & 2020 COMP 9
                    if (robotSettings.ENABLE_AGITATOR) {
                        if (robotSettings.ENABLE_INDEXER_AUTO_INDEX) {
                            agitator.moveAtPercent(!isIndexed() ? 0.5 : 0);
                        } else {
                            agitator.moveAtPercent(0);
                        }
                    }
                } else {
                    if (robotSettings.ENABLE_INDEXER) {
                        agitator.moveAtPercent(indexerActive ? 0.9 : 0);
                    }
                    if (robotSettings.ENABLE_AGITATOR) {
                        agitator.moveAtPercent(agitatorActive ? 0.6 : 0);
                    }
                }
                if (robotSettings.DEBUG && DEBUG) {
                    UserInterface.smartDashboardPutBoolean("indexer enable", indexerActive);
                    UserInterface.smartDashboardPutBoolean("agitator enable", agitatorActive);
                    UserInterface.smartDashboardPutNumber("indexer sensor", indexerSensorRange());
                    UserInterface.smartDashboardPutBoolean("hopper indexed", isIndexed());
                }
                break;
            }
            case STANDARD_2022: {
                if (!indexerActive && !agitatorActive && !agitatorTopbarActive) {
                    if (robotSettings.ENABLE_INDEXER) {
                        if (robotSettings.ENABLE_INDEXER_AUTO_INDEX) {
                            indexer.moveAtPercent(!isIndexed() ? 0.15 : 0);
                        } else {
                            indexer.moveAtPercent(0);
                        }
                    }
                    if (robotSettings.ENABLE_AGITATOR) {
                        if (panel.get(ControllerEnums.ButtonPanelButtons.HOPPER_IN) == ControllerEnums.ButtonStatus.DOWN) {
                            agitator.moveAtPercent(0.5);
                        } else if (panel.get(ControllerEnums.ButtonPanelButtons.HOPPER_OUT) == ControllerEnums.ButtonStatus.DOWN) {
                            agitator.moveAtPercent(-0.5);
                        } else if (robotSettings.ENABLE_INDEXER_AUTO_INDEX) {
                            agitator.moveAtPercent(!isIndexed() ? 0.3 : 0);
                        } else {
                            agitator.moveAtPercent(0);
                        }
                    }
                    if (robotSettings.ENABLE_AGITATOR_TOP) {
                        if (panel.get(ControllerEnums.ButtonPanelButtons.HOPPER_IN) == ControllerEnums.ButtonStatus.DOWN) {
                            agitatorTop.moveAtPercent(0.5);
                        } else if (panel.get(ControllerEnums.ButtonPanelButtons.HOPPER_OUT) == ControllerEnums.ButtonStatus.DOWN) {
                            agitatorTop.moveAtPercent(-0.5);
                        } else if (robotSettings.ENABLE_INDEXER_AUTO_INDEX) {
                            agitatorTop.moveAtPercent(!isIndexed() ? 0.4 : 0);
                        } else {
                            agitatorTop.moveAtPercent(0);
                        }
                    }
                } else {
                    if (robotSettings.ENABLE_INDEXER) {
                        indexer.moveAtPercent(indexerActive ? 0.6 : 0);
                    }
                    if (robotSettings.ENABLE_AGITATOR) {
                        agitator.moveAtPercent(agitatorActive ? 0.6 : 0);
                    }
                    if (robotSettings.ENABLE_AGITATOR_TOP) {
                        agitatorTop.moveAtPercent(agitatorTopbarActive ? 0.5 : 0);
                    }
                }
                if (robotSettings.DEBUG && DEBUG) {
                    UserInterface.smartDashboardPutBoolean("indexer enable", indexerActive);
                    UserInterface.smartDashboardPutBoolean("agitator enable", agitatorActive);
                    UserInterface.smartDashboardPutBoolean("agitator top enable", agitatorTopbarActive);
                    UserInterface.smartDashboardPutNumber("indexer sensor", indexerSensorRange());
                    UserInterface.smartDashboardPutBoolean("hopper indexed", isIndexed());
                }
                break;
            }
            case PRACTICE_2022: {
                setAll(controller.get(ControllerEnums.XBoxButtons.RIGHT_JOYSTICK_BUTTON) == ControllerEnums.ButtonStatus.DOWN);
                if (robotSettings.ENABLE_AGITATOR_TOP) agitatorTop.moveAtPercent(agitatorTopbarActive ? 0.5 : 0);
                if (robotSettings.ENABLE_AGITATOR) agitator.moveAtPercent(agitatorActive ? 0.5 : 0);
                if (robotSettings.ENABLE_INDEXER) indexer.moveAtPercent(indexerActive ? 0.5 : 0);
                break;
            }
        }
    }

    @Override
    public void initTest() {

    }

    @Override
    public void initTeleop() {

    }

    @Override
    public void initAuton() {

    }

    @Override
    public void initDisabled() {

    }

    @Override
    public void initGeneric() {

    }

    @Override
    public String getSubsystemName() {
        return "Hopper";
    }

    /**
     * applies settings/toggles Agitator and Indexer on/off
     *
     * @param set a boolean to determine wether or not Agitator and Indexer is turned on/off
     */
    public void setAll(boolean set) {
        setAgitator(set);
        setAgitatorTopbar(set);
        setIndexer(set);
    }

    /**
     * applies settings/toggles Agitator on/off
     *
     * @param set a boolean to determine wether or not Agitator is turned on/off
     */
    public void setAgitator(boolean set) {
        agitatorActive = set;
        if (robotSettings.DEBUG && DEBUG) {
            System.out.println("Agitator set to " + set);
        }
    }

    /**
     * applies settings/toggles Agitator on/off
     *
     * @param set a boolean to determine wether or not Agitator is turned on/off
     */
    public void setAgitatorTopbar(boolean set) {
        agitatorTopbarActive = set;
        if (robotSettings.DEBUG && DEBUG) {
            System.out.println("Agitator Topbar set to " + set);
        }
    }

    /**
     * applies settings/toggles Indexer on/off
     *
     * @param set a boolean to determine wether or not Indexer is turned on/off
     */
    public void setIndexer(boolean set) {
        if (robotSettings.DEBUG && DEBUG) {
            System.out.println("Indexer set to " + set);
        }
        indexerActive = set;
    }

    /**
     * Used to change how the input is handled by the {@link Shooter} and what kind of controller to use
     */
    public enum HopperControlStyles {
        STANDARD, STANDARD_2022, PRACTICE_2022;

        private static SendableChooser<Shooter.ShootingControlStyles> myChooser;

        public static SendableChooser<Shooter.ShootingControlStyles> getSendableChooser() {
            return Objects.requireNonNullElseGet(myChooser, () -> {
                myChooser = new SendableChooser<>();
                for (Shooter.ShootingControlStyles style : Shooter.ShootingControlStyles.values())
                    myChooser.addOption(style.name(), style);
                return myChooser;
            });
        }
    }
}