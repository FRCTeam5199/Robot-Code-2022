package frc.motors;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import org.jetbrains.annotations.Nullable;

import static frc.robot.Robot.robotSettings;

/**
 * A nice wrapper to hold a {@link #driver driving motor} and a {@link #steering steering motor}. Main util method is
 * {@link #getState()} which is used in {@link frc.telemetry.RobotTelemetrySwivel telem}
 */
public class SwerveMotorController {
    public AbstractMotorController driver, steering;

    /**
     * Creates a new swerve drive module with any motors in {@link AbstractMotorController.SupportedMotors#values()}
     *
     * @param driverID          the id of the driver motor to instantiate
     * @param driverMotorType   the type of motor to create. If null, then no driving will be made
     * @param steeringID        the id of the steering motor to instantiate
     * @param steeringMotorType the type of motor to create. If null, then no steering will be made
     */
    public SwerveMotorController(int driverID, @Nullable AbstractMotorController.SupportedMotors driverMotorType, int steeringID, @Nullable AbstractMotorController.SupportedMotors steeringMotorType) {
        if (driverMotorType != null) {
            switch (driverMotorType) {
                case VICTOR:
                    driver = new VictorMotorController(driverID);
                    break;
                case TALON_FX:
                    driver = new TalonMotorController(robotSettings.DRIVE_MOTOR_CANBUS, driverID);
                    break;
                case CAN_SPARK_MAX:
                    driver = new SparkMotorController(driverID);
            }
            driver.setCurrentLimit(30);
        }
        if (steeringMotorType != null) {
            switch (steeringMotorType) {
                case VICTOR:
                    steering = new VictorMotorController(steeringID);
                    break;
                case TALON_FX:
                    steering = new TalonMotorController(robotSettings.DRIVE_MOTOR_CANBUS, steeringID);
                    break;
                case CAN_SPARK_MAX:
                    steering = new SparkMotorController(steeringID);
            }
            steering.setCurrentLimit(30);
        }
    }

    /**
     * The main reason to use this object.
     *
     * @return the module state of the two swerve motors
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                (driver.getSpeed()),
                Rotation2d.fromDegrees(steering.getRotations()));
    }
}
