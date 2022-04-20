package frc.ballstuff.shooting;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import frc.ballstuff.intaking.Hopper2020;
import frc.climber.Climber;
import frc.controllers.BaseController;
import frc.controllers.ControllerEnums;
import frc.controllers.ControllerEnums.ButtonPanelButtons;
import frc.controllers.ControllerEnums.ButtonStatus;
import frc.controllers.ControllerEnums.JoystickButtons;
import frc.misc.ISubsystem;
import frc.misc.PID;
import frc.misc.SubsystemStatus;
import frc.misc.UserInterface;
import frc.motors.AbstractMotorController;
import frc.motors.SparkMotorController;
import frc.motors.TalonMotorController;
import frc.robot.Robot;
import frc.selfdiagnostics.MotorDisconnectedIssue;
import frc.sensors.camera.IVision;
import frc.sensors.colorsensor.IColorSensor;
import frc.sensors.colorsensor.RevColorSensor;

import java.util.Objects;

import static frc.misc.UtilFunctions.weightedAverage;
import static frc.robot.Robot.*;

/**
 * Shooter pertains to spinning the flywheel that actually makes the balls go really fast (or slow if you don't like
 * fast things)
 */
public class Shooter implements ISubsystem {
    public static final boolean DEBUG = true;
    private final NetworkTableEntry P = UserInterface.SHOOTER_P.getEntry(),
            I = UserInterface.SHOOTER_I.getEntry(),
            D = UserInterface.SHOOTER_D.getEntry(),
            F = UserInterface.SHOOTER_F.getEntry(),
            constSpeed = UserInterface.SHOOTER_CONST_SPEED.getEntry(),
            calibratePID = UserInterface.SHOOTER_CALIBRATE_PID.getEntry(),
            rpmGraph = UserInterface.SHOOTER_RPM_GRAPH.getEntry(),
            backspinGraph = UserInterface.BACKSPIN_RPM_GRAPH.getEntry(),
            rpm = UserInterface.SHOOTER_RPM.getEntry(),
            BACKSPIN_P = UserInterface.BACKSPIN_P.getEntry(),
            BACKSPIN_I = UserInterface.BACKSPIN_I.getEntry(),
            BACKSPIN_D = UserInterface.BACKSPIN_D.getEntry(),
            BACKSPIN_F = UserInterface.BACKSPIN_F.getEntry(),
            calibrateBackspinPID = UserInterface.BACKSPIN_CALIBRATE_PID.getEntry(),
            constSpeedBackspinMult = UserInterface.BACKSPIN_CONST_SPEED_MULT.getEntry();
    public double speed = 4200;
    public double goalTicks = 20 * 15; //20 ticks = 1 second
    public int ballsShot = 0, ticksPassed = 0, emptyIndexerTicks = 0, hopperCooldownTicks = 0, ballsToShoot = 0;
    public int timerTicks = 0;
    public IVision goalCamera;
    public IColorSensor colorSensor;
    public AbstractMotorController leader, follower, backSpin;
    public boolean isConstSpeed, isConstSpeedLast = false, shooting = false, isSpinningUp = false, isSpinningUpHeld, singleShot = false, multiShot = false, loadingIndexer = false;
    public boolean checkForDips = false;
    public boolean tryFiringBalls = false;
    BaseController panel, joystickController, xbox;
    private PID lastPID = PID.EMPTY_PID;
    private PID backspinLastPID = PID.EMPTY_PID;
    public double backspinMult = 1.625;
    public double distanceFromGoal = 0;

    public Shooter() {
        addToMetaList();
        init();
    }

    /**
     * Initialize the Shooter object including the controller and the cameras and timers
     */
    @Override
    public void init() throws IllegalStateException {
        if (robotSettings.ENABLE_COLOR_SENSOR) {
            colorSensor = new RevColorSensor(I2C.Port.kOnboard);
            System.out.println("Enabling color sensor.");
        }

        switch (robotSettings.SHOOTER_CONTROL_STYLE) {
            case STANDARD_2022:
            case ACCURACY_2021:
            case SPEED_2021:
            case EXPERIMENTAL_OFFSEASON_2021:
            case STANDARD_OFFSEASON_2021:
            case BACKSPIN_SHOOT_2022:
            case STANDARD:
                joystickController = BaseController.createOrGet(robotSettings.FLIGHT_STICK_USB_SLOT, BaseController.Controllers.JOYSTICK_CONTROLLER);
                panel = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT, BaseController.Controllers.BUTTON_PANEL_CONTROLLER);
                xbox = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT, BaseController.Controllers.XBOX_CONTROLLER);
                break;
            case BOP_IT:
                joystickController = BaseController.createOrGet(3, BaseController.Controllers.BOP_IT_CONTROLLER);
                break;
            case PRACTICE_2022:
            case XBOX_CONTROLLER:
                joystickController = BaseController.createOrGet(0, BaseController.Controllers.XBOX_CONTROLLER);
                break;
            case FLIGHT_STICK:
                joystickController = BaseController.createOrGet(robotSettings.FLIGHT_STICK_USB_SLOT, BaseController.Controllers.JOYSTICK_CONTROLLER);
                break;
            case COMP_2022:
                panel = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT, BaseController.Controllers.BUTTTON_PANEL_CONTROLLER_2022);
                joystickController = BaseController.createOrGet(robotSettings.FLIGHT_STICK_USB_SLOT, BaseController.Controllers.JOYSTICK_CONTROLLER);
                xbox = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT, BaseController.Controllers.XBOX_CONTROLLER);
                break;
            default:
                throw new IllegalStateException("There is no UI configuration for " + robotSettings.SHOOTER_CONTROL_STYLE.name() + " to control the shooter. Please implement me");
        }
        createAndInitMotors();
        if (robotSettings.ENABLE_VISION) {
            goalCamera = IVision.manufactureGoalCamera(robotSettings.GOAL_CAMERA_TYPE);
        }
        backspinMult = robotSettings.BACKSPIN_MULTIPLIER;
    }

    @Override
    public SubsystemStatus getSubsystemStatus() {
        return !leader.isFailed() && !follower.isFailed() ? SubsystemStatus.NOMINAL : SubsystemStatus.FAILED;
    }

    /**
     * @see #updateGeneric()
     */
    @Override
    public void updateTest() {
        //goalCamera.setLedMode(IVision.VisionLEDMode.ON);
        switch (robotSettings.SHOOTER_CONTROL_STYLE) {
            case STANDARD_2022: {
                goalCamera.setLedMode((panel.get(ButtonPanelButtons.BUDDY_CLIMB) == ButtonStatus.DOWN) ? IVision.VisionLEDMode.ON : IVision.VisionLEDMode.OFF);
            }
            break;
            /*default:
                throw new IllegalStateException("There is no UI configuration for " + robotSettings.SHOOTER_CONTROL_STYLE.name() + " to control the shooter. Please implement me");
            */
        }
    }

    /**
     * @see #updateGeneric()
     */
    @Override
    public void updateTeleop() {
        updateGeneric();
    }

    /**
     * Creates and initializes all the controllers you might use in the shooter
     */
    public void createControllers() {
        switch (robotSettings.SHOOTER_CONTROL_STYLE) {
            case ACCURACY_2021:
            case SPEED_2021:
            case EXPERIMENTAL_OFFSEASON_2021:
            case STANDARD_OFFSEASON_2021:
            case BACKSPIN_SHOOT_2022:
            case STANDARD:
                panel = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT, BaseController.Controllers.BUTTON_PANEL_CONTROLLER);
                xbox = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT, BaseController.Controllers.XBOX_CONTROLLER);
                joystickController = BaseController.createOrGet(robotSettings.FLIGHT_STICK_USB_SLOT, BaseController.Controllers.JOYSTICK_CONTROLLER);
            case COMP_2022:
                panel = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT, BaseController.Controllers.BUTTTON_PANEL_CONTROLLER_2022);
                xbox = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT, BaseController.Controllers.XBOX_CONTROLLER);
                joystickController = BaseController.createOrGet(robotSettings.FLIGHT_STICK_USB_SLOT, BaseController.Controllers.JOYSTICK_CONTROLLER);
            case FLIGHT_STICK:
                joystickController = BaseController.createOrGet(robotSettings.FLIGHT_STICK_USB_SLOT, BaseController.Controllers.JOYSTICK_CONTROLLER);
                break;
            case BOP_IT:
                joystickController = BaseController.createOrGet(3, BaseController.Controllers.BOP_IT_CONTROLLER);
                break;
            case PRACTICE_2022:
            case STANDARD_2022:
            case XBOX_CONTROLLER:
                joystickController = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT, BaseController.Controllers.XBOX_CONTROLLER);
                break;
            case DRUM_TIME:
                joystickController = BaseController.createOrGet(5, BaseController.Controllers.DRUM_CONTROLLER);
                break;
            case WII:
                joystickController = BaseController.createOrGet(4, BaseController.Controllers.WII_CONTROLLER);
                break;
            case GUITAR:
                joystickController = BaseController.createOrGet(6, BaseController.Controllers.SIX_BUTTON_GUITAR_CONTROLLER);
                break;
            default:
                throw new IllegalStateException("There is no UI configuration for " + robotSettings.SHOOTER_CONTROL_STYLE.name() + " to control the shooter. Please implement me");
        }
    }

    /**
     * Throws information about the shooter onto the shuffleboard, such as PID configuration, a speed graph, and a few
     * other important things.
     *
     * @author Smaltin
     */
    private void updateShuffleboard() {
        if (calibratePID.getBoolean(false) || calibrateBackspinPID.getBoolean(false)) {
            if (calibratePID.getBoolean(false)) {
                PID readPid = new PID(P.getDouble(robotSettings.SHOOTER_PID.getP()), I.getDouble(robotSettings.SHOOTER_PID.getI()), D.getDouble(robotSettings.SHOOTER_PID.getD()), F.getDouble(robotSettings.SHOOTER_PID.getF()));
                if (!lastPID.equals(readPid)) {
                    lastPID = readPid;
                    leader.setPid(lastPID);
                    if (robotSettings.DEBUG && DEBUG) {
                        System.out.println("Set shooter pid to " + lastPID);
                    }
                }
            }
            if (robotSettings.ENABLE_SHOOTER_BACKSPIN) {
                if (calibrateBackspinPID.getBoolean(false)) {
                    PID backReadPid = new PID(BACKSPIN_P.getDouble(robotSettings.BACKSPIN_PID.getP()), BACKSPIN_I.getDouble(robotSettings.BACKSPIN_PID.getI()), BACKSPIN_D.getDouble(robotSettings.BACKSPIN_PID.getD()), BACKSPIN_F.getDouble(robotSettings.BACKSPIN_PID.getF()));
                    if (!backspinLastPID.equals(backReadPid)) {
                        backspinLastPID = backReadPid;
                        backSpin.setPid(backspinLastPID);
                        if (robotSettings.DEBUG && DEBUG) {
                            System.out.println("Set shooter backspin pid to " + backspinLastPID);
                        }
                    }
                }
            }
        } else {
            if (!isConstSpeed && isConstSpeedLast) {
                leader.setPid(robotSettings.SHOOTER_PID);
                if (robotSettings.ENABLE_SHOOTER_BACKSPIN)
                    backSpin.setPid(robotSettings.BACKSPIN_PID);
                isConstSpeedLast = false;
                if (DEBUG && robotSettings.DEBUG) {
                    System.out.println("Normal shooter PID.");
                }
            } else {
                if (DEBUG && robotSettings.DEBUG) {
                    System.out.println("Running constant speed PID.");
                }
            }
        }
        //UserInterface.smartDashboardPutNumber("RPM", leader.getSpeed());
        rpmGraph.setNumber(leader.getSpeed());
        rpm.setNumber(leader.getSpeed());
        if (robotSettings.TARGET_HEIGHT != 0 && robotSettings.ENABLE_VISION) {
            if (isValidTarget()) {
                switch (robotSettings.SHOOTER_CONTROL_STYLE) {
                    case COMP_2022:
                    case STANDARD_2022:
                    case PRACTICE_2022:
                        distanceFromGoal = goalCamera.getDistanceUsingPitch();
                        break;
                    case STANDARD_OFFSEASON_2021:
                        distanceFromGoal = goalCamera.getDistanceUsingPitch();
                        break;
                }
                UserInterface.smartDashboardPutNumber("Distance from Target", distanceFromGoal);
            } else {
                //distanceFromGoal = -1;
            }
        }
        if (robotSettings.ENABLE_SHOOTER_BACKSPIN) {
            backspinGraph.setNumber(backSpin.getSpeed());
            UserInterface.smartDashboardPutNumber("Current BackSpin RPM", backSpin.getSpeed());
            UserInterface.smartDashboardPutNumber("Backspin Target RPM", speed * backspinMult);
        }
        UserInterface.smartDashboardPutNumber("Target RPM", speed);
        UserInterface.smartDashboardPutBoolean("atSpeed", isAtSpeed());
        UserInterface.smartDashboardPutBoolean("IS SHOOTING?", shooting);
    }

    /**
     * Runs the default update which unsets hopper2020 {@link Hopper2020#setAll(boolean) active flags}
     * and sets speed to constant speed (or 0)
     */
    private void shooterDefault() {
        if (robotSettings.ENABLE_2020_HOPPER) {
            hopper2020.setAll(false);
        }
        if (robotSettings.ENABLE_HOPPER)
            hopper.setAll(false);
        double speedYouWant = constSpeed.getDouble(0);
        if (speedYouWant != 0) {
            isConstSpeed = true;
            if (!isConstSpeedLast) {
                isConstSpeedLast = true;
                leader.setPid(robotSettings.SHOOTER_CONST_SPEED_PID);
            }
            leader.moveAtVelocity(speedYouWant);
            if (robotSettings.ENABLE_SHOOTER_BACKSPIN)
                backSpin.moveAtVelocity(speedYouWant * constSpeedBackspinMult.getDouble(robotSettings.BACKSPIN_MULTIPLIER));
        } else {
            leader.moveAtPercent(0);
        }
        shooting = false;
        ballsShot = 0;
    }

    /**
     * if the shooter is actually at the requested speed
     *
     * @return if the shooter is actually at the requested speed
     */
    public boolean isAtSpeed() {
        return robotSettings.ENABLE_SHOOTER_BACKSPIN ?
                Math.abs(leader.getSpeed() - speed) < 150
                        && Math.abs(backSpin.getSpeed() - (speed * backspinMult)) < 150
                : Math.abs(leader.getSpeed() - speed) < 150;
    }

    @Override
    public void updateAuton() {
        updateShuffleboard();
    }

    /**
     * Input is parsed and shooter object maintained appropriately.
     *
     * @throws IllegalStateException if control is not implemented for {@link frc.robot.robotconfigs.DefaultConfig#SHOOTER_CONTROL_STYLE
     *                               current control style}
     * @see frc.robot.robotconfigs.DefaultConfig#SHOOTER_CONTROL_STYLE
     * @see ShootingControlStyles
     */
    @Override
    public void updateGeneric() throws IllegalStateException {
        if (leader.getSpeed() > 1000) {
            if (robotSettings.ENABLE_DRIVE) {
                //xbox.rumble(0.1 * (leader.getSpeed() / 4200));
            }
        }
        if (ShootingControlStyles.getSendableChooser().getSelected() != null && robotSettings.SHOOTER_CONTROL_STYLE != ShootingControlStyles.getSendableChooser().getSelected()) {
            robotSettings.SHOOTER_CONTROL_STYLE = ShootingControlStyles.getSendableChooser().getSelected();
            if (Robot.turret != null)
                Robot.turret.updateControl();
            createControllers();
        }
        MotorDisconnectedIssue.handleIssue(this, leader, follower);
        updateShuffleboard();
        switch (robotSettings.SHOOTER_CONTROL_STYLE) {
            case STANDARD: {
                if (panel.get(ButtonPanelButtons.SOLID_SPEED) == ButtonStatus.DOWN) {
                    ShootingEnums.FIRE_SOLID_SPEED_FLIGHTSTICK.shoot(this);
                    isConstSpeed = false;
                } else if (panel.get(ButtonPanelButtons.TARGET) == ButtonStatus.DOWN && joystickController.get(JoystickButtons.ONE) == ButtonStatus.DOWN) {
                    ShootingEnums.FIRE_HIGH_SPEED.shoot(this);
                    isConstSpeed = false;
                } else {
                    if (robotSettings.ENABLE_2020_HOPPER) {
                        hopper2020.setAll(false);
                    }
                    leader.moveAtPercent(0);
                    shooting = false;
                    ballsShot = 0;
                }
                if (robotSettings.ENABLE_SHOOTER_COOLING) {
                    if (panel.get(ButtonPanelButtons.AUX_BOTTOM) == ButtonStatus.DOWN) { //OFF
                        pneumatics.shooterCooling.set(false);
                    } else if (panel.get(ButtonPanelButtons.AUX_TOP) == ButtonStatus.DOWN) { //ON
                        pneumatics.shooterCooling.set(true);
                    }
                }
                break;
            }
            case STANDARD_OFFSEASON_2021: {
                if (panel.get(ButtonPanelButtons.SOLID_SPEED) == ButtonStatus.DOWN) {
                    ShootingEnums.FIRE_SOLID_SPEED_FLIGHTSTICK.shoot(this);
                    isConstSpeed = false;
                } else if ((panel.get(ButtonPanelButtons.AUX_TOP) == ButtonStatus.DOWN || panel.get(ButtonPanelButtons.AUX_BOTTOM) == ButtonStatus.DOWN)) {
                    shooter.setSpeed(3700 + (500 * shooter.joystickController.getPositive(ControllerEnums.JoystickAxis.SLIDER)));
                    if (joystickController.get(JoystickButtons.ONE) == ButtonStatus.DOWN) {
                        ShootingEnums.FIRE_HIGH_SPEED_SPINUP.shoot(this);
                    } else {
                        shooter.setShooting(false);
                        shooter.tryFiringBalls = false;
                        hopper2020.setAll(false);
                    }
                } else if (panel.get(ButtonPanelButtons.TARGET) == ButtonStatus.DOWN && joystickController.get(JoystickButtons.ONE) == ButtonStatus.DOWN) {
                    tryFiringBalls = true;
                    if (articulatedHood.isAtWantedPosition) {
                        ShootingEnums.FIRE_HIGH_SPEED.shoot(this);
                        isConstSpeed = false;
                    }
                } else {
                    tryFiringBalls = false;
                    if (robotSettings.ENABLE_2020_HOPPER) {
                        hopper2020.setAll(false);
                    }
                    leader.moveAtPercent(0);
                    ballsShot = 0;
                    shooterDefault();
                }
                if (robotSettings.ENABLE_SHOOTER_COOLING) {
                    if (panel.get(ButtonPanelButtons.AUX_BOTTOM) == ButtonStatus.DOWN) { //OFF
                        pneumatics.shooterCooling.set(false);
                    } else if (panel.get(ButtonPanelButtons.AUX_TOP) == ButtonStatus.DOWN) { //ON
                        pneumatics.shooterCooling.set(true);
                    }
                }
                break;
            }
            case EXPERIMENTAL_OFFSEASON_2021: {
                if (panel.get(ButtonPanelButtons.SOLID_SPEED) == ButtonStatus.DOWN) {
                    if (!isSpinningUpHeld) {
                        isSpinningUp = !isSpinningUp;
                        isSpinningUpHeld = true;
                    }
                } else if (panel.get(ButtonPanelButtons.SOLID_SPEED) == ButtonStatus.UP) {
                    isSpinningUpHeld = false;
                }
                if (isSpinningUp) {
                    ShootingEnums.FIRE_SOLID_SPEED_OFFSEASON21.shoot(this);
                } else {
                    shooterDefault();
                }
                break;
            }
            case ACCURACY_2021: {
                if (Robot.articulatedHood.unTargeted) {
                    shooterDefault();
                } else if (panel.get(ControllerEnums.ButtonPanelTapedButtons.SOLID_SPEED) == ButtonStatus.DOWN) {
                    ShootingEnums.FIRE_SOLID_SPEED_FLIGHTSTICK.shoot(this);
                    isConstSpeed = false;
                } else if (singleShot) {
                    ShootingEnums.FIRE_SINGLE_SHOT.shoot(this);
                    isConstSpeed = false;
                } else if (panel.get(ControllerEnums.ButtonPanelTapedButtons.SINGLE_SHOT) == ButtonStatus.DOWN) {
                    singleShot = true;
                } else {
                    shooterDefault();
                }
                if (robotSettings.ENABLE_SHOOTER_COOLING) {
                    if (panel.get(ButtonPanelButtons.AUX_BOTTOM) == ButtonStatus.DOWN) { //OFF
                        pneumatics.shooterCooling.set(false);
                    } else if (panel.get(ButtonPanelButtons.AUX_TOP) == ButtonStatus.DOWN) { //ON
                        pneumatics.shooterCooling.set(true);
                    }
                }
                break;
            }
            case SPEED_2021: {
                if (panel.get(ButtonPanelButtons.TARGET) == ButtonStatus.DOWN && joystickController.get(JoystickButtons.ONE) == ButtonStatus.DOWN) {
                    ShootingEnums.FIRE_WITH_HOPPER_CONTROLLED.shoot(this);
                    //ShootingEnums.FIRE_TIMED.shoot(this);
                    isConstSpeed = false;
                } else if (panel.get(ButtonPanelButtons.HOPPER_IN) == ButtonStatus.DOWN) {
                    ShootingEnums.FIRE_SOLID_SPEED_FLIGHTSTICK.shoot(this);
                    isConstSpeed = false;
                } else {
                    //shooterDefault();
                }
                if (robotSettings.ENABLE_SHOOTER_COOLING) {
                    if (panel.get(ButtonPanelButtons.AUX_BOTTOM) == ButtonStatus.DOWN) { //OFF
                        pneumatics.shooterCooling.set(false);
                    } else if (panel.get(ButtonPanelButtons.AUX_TOP) == ButtonStatus.DOWN) { //ON
                        pneumatics.shooterCooling.set(true);
                    }
                }
                break;
            }
            case FLIGHT_STICK: {
                if (joystickController.get(JoystickButtons.TWO) == ButtonStatus.DOWN) {
                    ShootingEnums.FIRE_SOLID_SPEED_FLIGHTSTICK.shoot(this);
                } else if (joystickController.get(JoystickButtons.ONE) == ButtonStatus.DOWN) {
                    ShootingEnums.FIRE_HIGH_SPEED.shoot(this);
                    isConstSpeed = false;
                } else {
                    if (robotSettings.ENABLE_2020_HOPPER) {
                        hopper2020.setAll(false);
                    }
                    leader.moveAtPercent(0);
                    shooting = false;
                    ballsShot = 0;
                }
                break;
            }
            case BOP_IT: {
                if (joystickController.get(ControllerEnums.BopItButtons.PULLIT) == ButtonStatus.DOWN || singleShot) {
                    ShootingEnums.FIRE_SINGLE_SHOT.shoot(this);
                    isConstSpeed = false;
                } else {
                    shooting = false;
                    ballsShot = 0;
                    shooterDefault();
                }
                break;
            }
            case GUITAR: {
                if (joystickController.get(ControllerEnums.SixKeyGuitarButtons.HERO_POWER) == ButtonStatus.DOWN || singleShot) {
                    ShootingEnums.FIRE_SINGLE_SHOT.shoot(this);
                    isConstSpeed = false;
                } else {
                    shooting = false;
                    ballsShot = 0;
                    shooterDefault();
                }
                break;
            }

            case WII: {
                if (joystickController.get(ControllerEnums.WiiButton.TWO) == ButtonStatus.DOWN) {
                    ShootingEnums.FIRE_SOLID_SPEED_WII.shoot(this);
                    isConstSpeed = false;
                } else if (joystickController.get(ControllerEnums.WiiButton.ONE) == ButtonStatus.DOWN || singleShot) {
                    ShootingEnums.FIRE_SINGLE_SHOT.shoot(this);
                    isConstSpeed = false;
                } else {
                    shooting = false;
                    ballsShot = 0;
                    shooterDefault();
                }
            }

            case DRUM_TIME: {
                if (joystickController.get(ControllerEnums.DrumButton.A) == ButtonStatus.DOWN) {
                    ShootingEnums.FIRE_SOLID_SPEED_DRUMS.shoot(this);
                    isConstSpeed = false;
                } else if (joystickController.get(ControllerEnums.DrumButton.ONE) == ButtonStatus.DOWN || singleShot) {
                    ShootingEnums.FIRE_SINGLE_SHOT.shoot(this);
                    isConstSpeed = false;
                } else {
                    shooting = false;
                    ballsShot = 0;
                    shooterDefault();
                }
            }
            case XBOX_CONTROLLER: {
                if (joystickController.get(ControllerEnums.XboxAxes.RIGHT_TRIGGER) > 0.1) {
                    ShootingEnums.FIRE_SOLID_SPEED_XBOX_CONTROLLER.shoot(this);
                } else {
                    leader.moveAtPercent(0);
                    shooting = false;
                    ballsShot = 0;
                }
                break;
            }
            case STANDARD_2022: {
                if (panel.get(ButtonPanelButtons.SOLID_SPEED) == ButtonStatus.DOWN) {
                    ShootingEnums.FIRE_SOLID_SPEED_STANDARD2022.shoot(this);
                    //ShootingEnums.PID_TUNING.shoot(this);
                    isConstSpeed = false;
                } else if ((panel.get(ButtonPanelButtons.AUX_TOP) == ButtonStatus.DOWN || panel.get(ButtonPanelButtons.AUX_BOTTOM) == ButtonStatus.DOWN) && robotSettings.CLIMBER_CONTROL_STYLE != Climber.ClimberControlStyles.STANDARD_2022) {
                    shooter.setSpeed(1000 + (500 * shooter.joystickController.getPositive(ControllerEnums.JoystickAxis.SLIDER)));
                    if (joystickController.get(JoystickButtons.ONE) == ButtonStatus.DOWN) {
                        ShootingEnums.FIRE_HIGH_SPEED_SPINUP_2022.shoot(this);
                    } else {
                        shooter.setShooting(false);
                        shooter.tryFiringBalls = false;
                    }
                } else if (panel.get(ButtonPanelButtons.TARGET) == ButtonStatus.DOWN && joystickController.get(JoystickButtons.ONE) == ButtonStatus.DOWN) {
                    tryFiringBalls = true;
                    ShootingEnums.FIRE_HIGH_SPEED_2022.shoot(this);
                    isConstSpeed = false;

                } else {
                    tryFiringBalls = false;
                    leader.moveAtPercent(0);
                    ballsShot = 0;
                    shooterDefault();
                }
                break;
            }
            case BACKSPIN_SHOOT_2022: {
                if (joystickController.get(ControllerEnums.JoystickButtons.NINE) == ButtonStatus.DOWN || panel.get(ButtonPanelButtons.SOLID_SPEED) == ButtonStatus.DOWN) {
                    ShootingEnums.FIRE_SOLID_SPEED_BACKSPIN_2022.shoot(this);
                } else {
                    tryFiringBalls = false;
                    leader.moveAtPercent(0);
                    backSpin.moveAtPercent(0);
                    ballsShot = 0;
                    shooterDefault();
                }
                break;
            }
            case COMP_2022: {
                if (DEBUG && robotSettings.DEBUG && robotSettings.ENABLE_COLOR_SENSOR) {
                    System.out.println("I'm seeing color " + colorSensor.getColor().toString());
                }
                if (robotSettings.ENABLE_COLOR_SENSOR && !colorSensor.isColor(DriverStation.getAlliance() == DriverStation.Alliance.Red ? Color.kRed : Color.kBlue) && panel.get(ControllerEnums.ButtonPanelButtons2022.LOW_SHOT) == ButtonStatus.DOWN) {
                    ShootingEnums.WHAT_ARE_YOU_DOING_HERE_SHOO.shoot(this);
                } else if (panel.get(ControllerEnums.ButtonPanelButtons2022.FENDER_SHOT) == ButtonStatus.DOWN) {
                    if (robotSettings.ENABLE_SHOOTER_RPM_ARTICULATION) {
                        ShootingEnums.FIRE_FROM_RPM_ARTICULATION_2022.shoot(this);
                    } else {
                        ShootingEnums.FIRE_SOLID_SPEED_BACKSPIN_CLOSE_2022.shoot(this);
                    }
                } else if (panel.get(ControllerEnums.ButtonPanelButtons2022.TARMAC_SHOT) == ButtonStatus.DOWN) {
                    ShootingEnums.FIRE_SOLID_SPEED_BACKSPIN_MIDDLE_2022.shoot(this);
                } else if (panel.get(ControllerEnums.ButtonPanelButtons2022.FAR_SHOT) == ButtonStatus.DOWN) {
                    ShootingEnums.FIRE_SOLID_SPEED_BACKSPIN_FAR_2022.shoot(this);
                } else if (panel.get(ControllerEnums.ButtonPanelButtons2022.LOW_SHOT) == ButtonStatus.DOWN) {
                    if (robotSettings.ENABLE_SHOOTER_RPM_ARTICULATION) {
                        ShootingEnums.FIRE_FROM_RPM_ARTICULATION_2022.shoot(this);
                    } else {
                        ShootingEnums.FIRE_SOLID_SPEED_BACKSPIN_LOW_2022.shoot(this);
                    }
                } else {
                    tryFiringBalls = false;
                    leader.moveAtPercent(0);
                    backSpin.moveAtPercent(0);
                    ballsShot = 0;
                    shooterDefault();
                    if (robotSettings.ENABLE_PNOOMATICS && robotSettings.ENABLE_INDEXER_PISTON_BLOCK)
                        pneumatics.indexerBlocker.set(DoubleSolenoid.Value.kForward);
                }
                break;
            }
            case PRACTICE_2022: {
                //if (panel.get(ButtonPanelButtons.SOLID_SPEED) == ButtonStatus.DOWN) {
                if (joystickController.get(ControllerEnums.XBoxButtons.B_CIRCLE) == ButtonStatus.DOWN) {
                    ShootingEnums.FIRE_SOLID_SPEED_PRACTICE2022.shoot(this);
                    if (joystickController.get(ControllerEnums.XBoxButtons.RIGHT_JOYSTICK_BUTTON) == ButtonStatus.DOWN) {
                        hopper.setAll(true);
                    }
                } else {
                    hopper.setAll(false);
                    shooterDefault();
                }
                break;
            }
            default:
                throw new IllegalStateException("This UI not implemented for this controller");
        }
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

    }

    @Override
    public void initGeneric() {
        isConstSpeedLast = false;
        isConstSpeed = false;
        singleShot = false;
        if (robotSettings.SHOOTER_CONTROL_STYLE == ShootingControlStyles.SPEED_2021) {
            leader.setPid(new PID(0.0025, 0.0000007, 0.03, 0));
        } else {
            leader.setPid(robotSettings.SHOOTER_PID);
            if (robotSettings.ENABLE_SHOOTER_BACKSPIN) {
                backSpin.setPid(robotSettings.BACKSPIN_PID);
            }
        }
    }

    @Override
    public String getSubsystemName() {
        return "Shooter";
    }

    /**
     * Initialize the motors. Checks for {@link AbstractMotorController.SupportedMotors SHOOTER_MOTOR_TYPE} and {@link frc.robot.robotconfigs.DefaultConfig SHOOTER_USE_TWO_MOTORS} to allow modularity.
     *
     * @throws IllegalStateException If the motor configuration is not implemented
     */
    private void createAndInitMotors() throws IllegalStateException {
        switch (robotSettings.SHOOTER_MOTOR_TYPE) {
            case CAN_SPARK_MAX:
                leader = new SparkMotorController(robotSettings.SHOOTER_LEADER_ID);
                if (robotSettings.SHOOTER_USE_TWO_MOTORS) {
                    follower = new SparkMotorController(robotSettings.SHOOTER_FOLLOWER_ID);
                    follower.setSensorToRealDistanceFactor(1);
                }
                leader.setSensorToRealDistanceFactor(1);
                break;
            case TALON_FX:
                leader = new TalonMotorController(robotSettings.SHOOTER_MOTOR_CANBUS, robotSettings.SHOOTER_LEADER_ID);
                if (robotSettings.SHOOTER_USE_TWO_MOTORS) {
                    follower = new TalonMotorController(robotSettings.SHOOTER_MOTOR_CANBUS, robotSettings.SHOOTER_FOLLOWER_ID);
                    follower.setSensorToRealDistanceFactor(600 / robotSettings.CTRE_SENSOR_UNITS_PER_ROTATION);
                }
                if (robotSettings.ENABLE_SHOOTER_BACKSPIN) {
                    backSpin = new TalonMotorController(robotSettings.SHOOTER_BACKSPIN_MOTOR_CANBUS, robotSettings.BACKSPIN_ID);
                    backSpin.setSensorToRealDistanceFactor(600 / robotSettings.CTRE_SENSOR_UNITS_PER_ROTATION);
                }
                leader.setSensorToRealDistanceFactor(600 / robotSettings.CTRE_SENSOR_UNITS_PER_ROTATION);
                break;
            default:
                throw new IllegalStateException("No such supported shooter motor config for " + robotSettings.SHOOTER_MOTOR_TYPE.name());
        }

        leader.setInverted(robotSettings.SHOOTER_INVERTED);
        if (robotSettings.SHOOTER_USE_TWO_MOTORS) {
            follower.follow(leader, !robotSettings.SHOOTER_INVERTED).setCurrentLimit(80).setBrake(false);
        }
        if (robotSettings.ENABLE_SHOOTER_BACKSPIN) {
            backSpin.setInverted(robotSettings.BACKSPIN_INVERTED).setOpenLoopRampRate(3);
        }
        leader.setCurrentLimit(80).setBrake(false).setOpenLoopRampRate(1).resetEncoder();
        //leader.setOpenLoopRampRate(0);
        //follower.setOpenLoopRampRate(0);
    }

    /**
     * If the shooter is at the requested speed
     *
     * @param rpm how fast it should be going
     * @return if the shooter is at the requested speed
     */
    public boolean isAtSpeed(int rpm) {
        return Math.abs(leader.getSpeed() - rpm) < 200;
    }

    /**
     * if the goal photon is in use and has a valid target in its sights
     *
     * @return if the goal photon is in use and has a valid target in its sights
     */
    public boolean isValidTarget() {
        return robotSettings.ENABLE_VISION && goalCamera.hasValidTarget();
    }

    /**
     * Gets the current speed of the leader motor
     *
     * @return the current speed of the leader motor based on the output units of {@link #leader}
     */
    public double getSpeed() {
        return leader.getSpeed();
    }

    public double getSpeedToShoot() {
        if (distanceFromGoal >= robotSettings.CALIBRATED_SHOOTER_RPM_ARRAY[robotSettings.CALIBRATED_SHOOTER_RPM_ARRAY.length - 1][0]) { //high bound
            System.out.println("A " + robotSettings.CALIBRATED_SHOOTER_RPM_ARRAY[robotSettings.CALIBRATED_SHOOTER_RPM_ARRAY.length - 1][1]);
            return robotSettings.CALIBRATED_SHOOTER_RPM_ARRAY[robotSettings.CALIBRATED_SHOOTER_RPM_ARRAY.length - 1][1];
        }
        if (distanceFromGoal <= robotSettings.CALIBRATED_SHOOTER_RPM_ARRAY[0][0]) { //low bound
            System.out.println("B " + robotSettings.CALIBRATED_SHOOTER_RPM_ARRAY[0][1]);
            return robotSettings.CALIBRATED_SHOOTER_RPM_ARRAY[0][1];
        }
        for (int i = 1; i < robotSettings.CALIBRATED_SHOOTER_RPM_ARRAY.length; i++) {
            if (distanceFromGoal < robotSettings.CALIBRATED_SHOOTER_RPM_ARRAY[i][0]) {
                System.out.println("C " + weightedAverage(distanceFromGoal, robotSettings.CALIBRATED_SHOOTER_RPM_ARRAY[i], robotSettings.CALIBRATED_SHOOTER_RPM_ARRAY[i - 1]));
                return weightedAverage(distanceFromGoal, robotSettings.CALIBRATED_SHOOTER_RPM_ARRAY[i], robotSettings.CALIBRATED_SHOOTER_RPM_ARRAY[i - 1]);
            }
        }
        return 0;
    }

    /**
     * Set drive wheel RPM
     *
     * @param rpm speed to set
     */
    public void setSpeed(double rpm) {
        if (robotSettings.DEBUG && DEBUG) {
            System.out.println("set shooter speed to " + rpm);
        }
        speed = rpm;
        leader.moveAtVelocity(rpm);

        if (robotSettings.ENABLE_SHOOTER_BACKSPIN)
            backSpin.moveAtVelocity(rpm * backspinMult);
    }

    /**
     * Set drive wheel RPM
     *
     * @param rpm speed to set
     */
    public void setSpeed(double rpm, boolean useBackspin) {
        if (robotSettings.DEBUG && DEBUG) {
            System.out.println("set shooter speed to " + rpm);
        }
        speed = rpm;
        leader.moveAtVelocity(rpm);
        if (robotSettings.ENABLE_SHOOTER_BACKSPIN && useBackspin)
            backSpin.moveAtVelocity(rpm * backspinMult);
    }

    public void setSpeed(double rpm, double backSpinRPM) {
        if (robotSettings.DEBUG && DEBUG) {
            System.out.println("set shooter speed to " + rpm);
        }
        speed = rpm;
        leader.moveAtVelocity(rpm);
        if (robotSettings.ENABLE_SHOOTER_BACKSPIN)
            backSpin.moveAtVelocity(backSpinRPM);
    }

    /**
     * Getter for {@link #shooting}
     *
     * @return whether the shooter is shooting or not
     */
    public boolean isShooting() {
        return shooting;
    }

    void setShooting(boolean shooting) {
        this.shooting = shooting;
    }

    /**
     * Old method of firing a single shot out of the shooter. This is replaced by {@link Shooter#fireAmount(int)}
     *
     * @return true if the ball has been shot or false if the ball has not been shot yet
     */
    @Deprecated
    public boolean fireSingleShot() {
        singleShot = true;
        shooting = true;
        ShootingEnums.FIRE_SINGLE_SHOT.shoot(this);
        isConstSpeed = !singleShot;
        if (!singleShot) {
            shooting = false;
            setPercentSpeed(0);
            hopper2020.setAll(false);
        }
        updateShuffleboard();
        return !singleShot;
    }

    /**
     * Moves the shooter at a -1 to 1 percent speed
     *
     * @param percentSpeed percent from -1 to 1 to move the shooter at
     */
    public void setPercentSpeed(double percentSpeed) {
        speed = percentSpeed * leader.getMaxRPM() * robotSettings.SHOOTER_FLYWHEEL_WEIGHT_MULTIPLIER;
        leader.moveAtPercent(percentSpeed);
    }

    /**
     * Moves the shooter at a -1 to 1 percent speed
     *
     * @param percentSpeed percent from -1 to 1 to move the shooter at
     */
    public void setPercentSpeed(double percentSpeed, double backspinSpeed) {
        speed = percentSpeed * leader.getMaxRPM() * robotSettings.SHOOTER_FLYWHEEL_WEIGHT_MULTIPLIER;
        leader.moveAtPercent(percentSpeed);
        backSpin.moveAtPercent(backspinSpeed);
    }

    /**
     * Fires multiple balls without caring if it sees the target. Good for autonomous and the discord/slaque bot
     *
     * @param shots how many balls to shoot
     * @return if the balls have finished shooting
     * @author Smaltin
     */
    public boolean fireAmount(int shots) {
        ballsToShoot = shots;
        multiShot = true;
        ShootingEnums.FIRE_MULTIPLE_SHOTS.shoot(this);
        isConstSpeed = !multiShot;
        updateShuffleboard();
        if (!multiShot) {
            shooting = false;
            setPercentSpeed(0);
            hopper2020.setAll(false);
        }
        return !multiShot;
    }

    /**
     * Fires multiple balls without caring if it sees the target. Good for autonomous and the discord/slaque bot
     * Used for the 2022 ball shooter
     *
     * @param seconds how many seconds to run the shooter
     * @return if the balls have finished shooting
     * @author Smaltin
     */
    public boolean fireAmount2022(double seconds, int rpm) {
        speed = rpm;
        goalTicks = seconds * 50; //tick = 20ms. 50 ticks in a second.
        if (!shooting) {
            ticksPassed = 0;
            shooting = true;
            multiShot = true;
        }
        ShootingEnums.FIRE_TIMED_2022.shoot(this);
        updateShuffleboard();
        if (!multiShot) {
            shooting = false;
            setPercentSpeed(0, 0);
            hopper.setAll(false);
            shooter.timerTicks = 0;
        }
        return !multiShot;
    }

    public boolean fireTimed(int seconds) {
        goalTicks = seconds * 50; //tick = 20ms. 50 ticks in a second.
        if (!shooting) {
            ticksPassed = 0;
            shooting = true;
            multiShot = true;
        }
        ShootingEnums.FIRE_TIMED.shoot(this);
        updateShuffleboard();
        if (!multiShot) {
            shooting = false;
            setPercentSpeed(0);
            hopper2020.setAll(false);
        }
        return !multiShot;
    }

    /**
     * Used to change how the input is handled by the {@link Shooter} and what kind of controller to use
     */
    public enum ShootingControlStyles {
        STANDARD, BOP_IT, XBOX_CONTROLLER, ACCURACY_2021, SPEED_2021, STANDARD_2020, EXPERIMENTAL_OFFSEASON_2021, STANDARD_OFFSEASON_2021, WII, DRUM_TIME, GUITAR, FLIGHT_STICK, PRACTICE_2022, STANDARD_2022, BACKSPIN_SHOOT_2022, COMP_2022;

        private static SendableChooser<ShootingControlStyles> myChooser;

        public static SendableChooser<ShootingControlStyles> getSendableChooser() {
            return Objects.requireNonNullElseGet(myChooser, () -> {
                myChooser = new SendableChooser<>();
                for (ShootingControlStyles style : ShootingControlStyles.values())
                    myChooser.addOption(style.name(), style);
                return myChooser;
            });
        }
    }
}