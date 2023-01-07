package frc.ballstuff.intaking;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.controllers.basecontrollers.BaseController;
import frc.controllers.ControllerEnums;
import frc.controllers.basecontrollers.DefaultControllerEnums;
import frc.drive.auton.AutonType;
import frc.misc.*;
import frc.motors.*;
import frc.robot.Robot;
import frc.selfdiagnostics.MotorDisconnectedIssue;
import frc.sensors.BreakBeamSensor;
import frc.sensors.LimitSwitchSensor;

import java.util.Objects;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.controllers.ControllerEnums.ButtonPanelButtons.INTAKE_DOWN;
import static frc.controllers.ControllerEnums.ButtonPanelButtons.INTAKE_UP;
import static frc.robot.Robot.robotSettings;

/**
 * The "Intake" is referring to the part that picks up power cells from the floor
 */
public class Intake implements ISubsystem {
    private static final boolean DEBUG = false;
    public AbstractMotorController intakeMotor;
    public Servo intakeServo1;
    public Servo intakeServo2;
    public BaseController xbox, joystick, buttonpanel;
    public double intakeMult;
    public boolean isIntakeUp = false;
    public BreakBeamSensor intakeBreakBeam;
    public LimitSwitchSensor leftSensor, rightSensor;

    public Intake() throws InitializationFailureException, IllegalStateException {
        addToMetaList();
        init();
    }

    /**
     * create controller and motor
     *
     * @throws IllegalStateException if the selected control is not implemented
     */
    @Override
    public void init() throws IllegalStateException {
        createControllers();
        createMotors();
        createServos();
        if (robotSettings.ENABLE_INTAKE_RUMBLE_BREAK_BEAM && robotSettings.ENABLE_BREAK_BEAM) {
            intakeBreakBeam = new BreakBeamSensor(robotSettings.INTAKE_BREAK_BEAM_ID);
        }
        if (robotSettings.ENABLE_INTAKE_RUMBLE_LIMIT_SWITCH) {
            leftSensor = new LimitSwitchSensor(robotSettings.INTAKE_RUMBLE_L_ID);
            rightSensor = new LimitSwitchSensor(robotSettings.INTAKE_RUMBLE_R_ID);
        }
    }

    @Override
    public SubsystemStatus getSubsystemStatus() {
        return intakeMotor.isFailed() ? SubsystemStatus.FAILED : SubsystemStatus.NOMINAL;
    }

    /**
     * @see #updateGeneric()
     */
    @Override
    public void updateTest() {

    }

    /**
     * @see #updateGeneric()
     */
    @Override
    public void updateTeleop() {
        updateGeneric();
    }

    @Override
    public void updateAuton() {
        if (robotSettings.AUTON_TYPE == AutonType.GALACTIC_SEARCH || robotSettings.AUTON_TYPE == AutonType.GALACTIC_SCAM) {
            setIntake(robotSettings.autonComplete ? IntakeDirection.OFF : IntakeDirection.IN);
        }
        intakeMotor.moveAtPercent(.65 * intakeMult);
        // intakeMotor.moveAtPercent(.6 * intakeMult);
    }

    @Override
    public void updateGeneric() {
        if (IntakeControlStyles.getSendableChooser().getSelected() != null && robotSettings.INTAKE_CONTROL_STYLE != IntakeControlStyles.getSendableChooser().getSelected()) {
            robotSettings.INTAKE_CONTROL_STYLE = IntakeControlStyles.getSendableChooser().getSelected();
            createControllers();
        }
        MotorDisconnectedIssue.handleIssue(this, intakeMotor);
        intakeMotor.moveAtPercent(.65 * intakeMult);
        // intakeMotor.moveAtPercent(.6 * intakeMult);
        if (/*robotSettings.DEBUG && DEBUG*/ true) {
            UserInterface.smartDashboardPutNumber("Intake Speed", intakeMult);
           // System.out.println(intakeMult + "intake mult");
        }
        double speed;
        long RumbleCountdownEnd = 0;
        switch (robotSettings.INTAKE_CONTROL_STYLE) {
            case FLIGHT_STICK:
            case STANDARD:
                if (joystick.hatIs(DefaultControllerEnums.ResolvedCompassInput.DOWN)) {//|| buttonPanel.get(ControllerEnums.ButtonPanelButtons.) {
                    setIntake(IntakeDirection.IN);
                } else if (joystick.hatIs(DefaultControllerEnums.ResolvedCompassInput.UP)) {
                    setIntake(IntakeDirection.OUT);
                } else {
                    setIntake(IntakeDirection.OFF);
                }
                doIntakeArticulation();
                break;
            case ROBOT_2021:
                if (robotSettings.ENABLE_INTAKE_SERVOS) {
                    //do servo-y things
                    if (joystick.hatIs(DefaultControllerEnums.ResolvedCompassInput.DOWN)) {//|| buttonPanel.get(ControllerEnums.ButtonPanelButtons.) {
                        setIntake(IntakeDirection.IN);
                    } else if (joystick.hatIs(DefaultControllerEnums.ResolvedCompassInput.UP)) {
                        setIntake(IntakeDirection.OUT);
                    } else {
                        setIntake(IntakeDirection.OFF);
                    }

                    if (joystick.get(DefaultControllerEnums.JoystickButtons.FOUR) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        intakeServo1.moveToAngle(145);
                        intakeServo2.moveToAngle(35);
                    } else if (joystick.get(DefaultControllerEnums.JoystickButtons.SIX) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        intakeServo1.moveToAngle(0);
                        intakeServo2.moveToAngle(180);
                    }
                } else {
                    throw new IllegalStateException("You're unable to use the intake style ROBOT_2021 without a Servo as your motor type.");
                }
                break;
            case ROBOT_PRACTICE_2022: {
                if (joystick.get(DefaultControllerEnums.XBoxButtons.Y_TRIANGLE) == DefaultControllerEnums.ButtonStatus.DOWN) {//|| buttonPanel.get(ControllerEnums.ButtonPanelButtons.) {
                    setIntake(IntakeDirection.IN);
                } else {
                    setIntake(IntakeDirection.OFF);
                }
                doIntakeArticulation();
                break;
            }
            case ROBOT_2022_COMP:
                if (robotSettings.ENABLE_BREAK_BEAM && robotSettings.ENABLE_INTAKE_RUMBLE_BREAK_BEAM) {
                    xbox.rumble(intakeBreakBeam.isTriggered() ? 1 : 0);
                } else if (robotSettings.ENABLE_INTAKE_RUMBLE_LIMIT_SWITCH) {
                    boolean rumble = leftSensor.isTriggered() || rightSensor.isTriggered();
                    //xbox.rumble(rumble ? 1 : 0);
                    if(rumble) {
                        RumbleCountdownEnd = System.currentTimeMillis() + 500;
                    }
                    if (RumbleCountdownEnd >= System.currentTimeMillis()) {
                        xbox.rumble(1);
                    } else {
                        xbox.rumble(0);
                    }

                }
            case ROBOT_2022_OLD: {
                if (joystick.hatIs(DefaultControllerEnums.ResolvedCompassInput.DOWN)) {
                    setIntake(IntakeDirection.IN);
                    //hopper.setAgitatorTopbar(true);
                } else if (joystick.hatIs(DefaultControllerEnums.ResolvedCompassInput.UP)) {
                    setIntake(IntakeDirection.OUT);
                } else {
                    setIntake(IntakeDirection.OFF);
                    //hopper.setAgitatorTopbar(false);
                }
                doIntakeArticulation();
                break;
            }
            case DRUM_TIME:
                if (joystick.get(DefaultControllerEnums.DrumButton.TWO) == DefaultControllerEnums.ButtonStatus.DOWN)
                    setIntake(IntakeDirection.IN);
                else
                    setIntake(IntakeDirection.OFF);
                break;
            case BOP_IT:
                if (joystick.get(DefaultControllerEnums.BopItButtons.PULLIT) == DefaultControllerEnums.ButtonStatus.DOWN)
                    setIntake(IntakeDirection.IN);
                else
                    setIntake(IntakeDirection.OFF);
                break;
            case WII:
                speed = joystick.get(DefaultControllerEnums.WiiAxis.FORWARD_TILT);
                if (Math.abs(speed) >= 0.1) {
                    setIntake(speed);
                }
                break;
            case GUITAR:
                setIntake(joystick.get(DefaultControllerEnums.SixKeyGuitarAxis.STRUM));
                break;
            case XBOX_CONTROLLER:
                speed = joystick.get(DefaultControllerEnums.XboxAxes.RIGHT_TRIGGER);
                if (Math.abs(speed) >= 0.1) {
                    setIntake(speed);
                }
                break;
            default:
                throw new IllegalStateException("There is no UI configuration for " + robotSettings.INTAKE_CONTROL_STYLE.name() + " to control the shooter. Please implement me");
        }
        if (robotSettings.DEBUG && DEBUG) {
            UserInterface.smartDashboardPutNumber("Intake Speed", intakeMult);
        }
    }

    public void doIntakeArticulation() {
        switch (robotSettings.INTAKE_CONTROL_STYLE) {
            case ROBOT_2022_OLD:
                if (buttonpanel.get(INTAKE_UP) == DefaultControllerEnums.ButtonStatus.DOWN || joystick.get(DefaultControllerEnums.JoystickButtons.SIX) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    deployIntake(false);
                } else if (buttonpanel.get(INTAKE_DOWN) == DefaultControllerEnums.ButtonStatus.DOWN || joystick.get(DefaultControllerEnums.JoystickButtons.FOUR) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    deployIntake(true);
                }
                break;
            case ROBOT_2022_COMP:
                if (buttonpanel.get(ControllerEnums.ButtonPanelButtons2022.INTAKE_UP) == DefaultControllerEnums.ButtonStatus.DOWN || joystick.get(DefaultControllerEnums.JoystickButtons.SIX) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    deployIntake(false);
                } else if (buttonpanel.get(ControllerEnums.ButtonPanelButtons2022.INTAKE_DOWN) == DefaultControllerEnums.ButtonStatus.DOWN || joystick.get(DefaultControllerEnums.JoystickButtons.FOUR) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    deployIntake(true);
                }
                break;
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
        return "Intake";
    }

    /**
     * Set intake direction from {@link IntakeDirection enumeration}
     *
     * @param input desired {@link IntakeDirection direction}
     */
    public void setIntake(IntakeDirection input) {
        intakeMult = input.ordinal() - 1;
    }

    public void deployIntake(boolean deployed) {
        if (robotSettings.ENABLE_PNOOMATICS)
            Robot.pneumatics.solenoidIntake.set(deployed ? Value.kForward : Value.kReverse);
        isIntakeUp = deployed;
    }

    /**
     * Sets intake power and direction
     *
     * @param multiplier continuous direction desired; -1  for out, 1 for in, 0 for off
     */
    public void setIntake(double multiplier) {
        intakeMult = Math.min(1, Math.max(-1, multiplier));
    }

    private void createControllers() {
        switch (robotSettings.INTAKE_CONTROL_STYLE) {
            case FLIGHT_STICK:
            case ROBOT_PRACTICE_2022:
                joystick = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT, BaseController.DefaultControllers.XBOX_CONTROLLER);
                break;
            case ROBOT_2021:
                joystick = BaseController.createOrGet(robotSettings.FLIGHT_STICK_USB_SLOT, BaseController.DefaultControllers.JOYSTICK_CONTROLLER);
                break;
            case ROBOT_2022_COMP:
                xbox = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT, BaseController.DefaultControllers.XBOX_CONTROLLER);
                joystick = BaseController.createOrGet(robotSettings.FLIGHT_STICK_USB_SLOT, BaseController.DefaultControllers.JOYSTICK_CONTROLLER);
                buttonpanel = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT, ControllerEnums.CustomControllers.BUTTTON_PANEL_CONTROLLER_2022);
                break;
            case ROBOT_2022_OLD:
            case STANDARD:
                joystick = BaseController.createOrGet(robotSettings.FLIGHT_STICK_USB_SLOT, BaseController.DefaultControllers.JOYSTICK_CONTROLLER);
                buttonpanel = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT, ControllerEnums.CustomControllers.BUTTON_PANEL_CONTROLLER);
                break;
            case XBOX_CONTROLLER:
                joystick = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT, BaseController.DefaultControllers.XBOX_CONTROLLER);
                break;
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

    private void createMotors() {
        double s2rf;
        switch (robotSettings.INTAKE_MOTOR_TYPE) {
            case CAN_SPARK_MAX:
                intakeMotor = new SparkMotorController(robotSettings.INTAKE_MOTOR_ID, kBrushless);
                s2rf = 1;
                break;
            case TALON_FX:
                intakeMotor = new TalonMotorController(robotSettings.INTAKE_MOTOR_CANBUS, robotSettings.INTAKE_MOTOR_ID);
                s2rf = 600.0 / robotSettings.CTRE_SENSOR_UNITS_PER_ROTATION;
                break;
            case VICTOR:
                intakeMotor = new VictorMotorController(robotSettings.INTAKE_MOTOR_ID);
                s2rf = 600.0 / robotSettings.CTRE_SENSOR_UNITS_PER_ROTATION;
                break;
            default:
                throw new InitializationFailureException("DriveManager does not have a suitible constructor for " + robotSettings.DRIVE_MOTOR_TYPE.name(), "Add an implementation in the init for drive manager");
        }
        intakeMotor.setSensorToRealDistanceFactor(s2rf);
        intakeMotor.setInverted(robotSettings.INTAKE_INVERT_MOTOR);
        intakeMotor.setBrake(false);
    }


    private void createServos() {
        if (robotSettings.ENABLE_INTAKE_SERVOS) {
            intakeServo1 = new Servo(robotSettings.INTAKE_SERVO_L_ID);
            intakeServo2 = new Servo(robotSettings.INTAKE_SERVO_R_ID);
        }
    }

    /**
     * Preserve this order. Out runs the motor at 0 - 1 = -1, off at 1 - 1 = 0, and in at 2 - 1 = 1 (percent)
     *
     * @see #setIntake(IntakeDirection)
     */
    public enum IntakeDirection {
        OUT, OFF, IN
    }

    /**
     * Determines how the {@link Intake} uses user input and what controllers to use
     */
    public enum IntakeControlStyles {
        STANDARD,
        ROBOT_2021,
        ROBOT_PRACTICE_2022,
        ROBOT_2022_OLD,
        WII,
        DRUM_TIME,
        GUITAR,
        BOP_IT,
        FLIGHT_STICK,
        XBOX_CONTROLLER,
        ROBOT_2022_COMP;

        private static SendableChooser<IntakeControlStyles> myChooser;

        public static SendableChooser<IntakeControlStyles> getSendableChooser() {
            return Objects.requireNonNullElseGet(myChooser, () -> {
                myChooser = new SendableChooser<>();
                for (IntakeControlStyles style : IntakeControlStyles.values())
                    myChooser.addOption(style.name(), style);
                return myChooser;
            });
        }
    }
}