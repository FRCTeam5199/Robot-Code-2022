package frc.ballstuff.intaking;

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

import static com.revrobotics.Rev2mDistanceSensor.Port.kOnboard;
import static com.revrobotics.Rev2mDistanceSensor.RangeProfile.kHighAccuracy;
import static com.revrobotics.Rev2mDistanceSensor.Unit.kInches;
import static frc.robot.Robot.robotSettings;

/**
 * The Hopper2020 subsystem effectively takes a ball from the front (where the {@link frc.ballstuff.intaking.Intake intake}
 * is ) to the {@link frc.ballstuff.shooting.Shooter}
 */
public class Hopper2020 implements ISubsystem {
    private static final boolean DEBUG = false;
    public AbstractMotorController agitator, indexer;
    public IDistanceSensor indexSensor;
    public boolean agitatorActive = false, indexerActive = false;

    public Hopper2020() {
        addToMetaList();
        init();
    }

    @Override
    public void init() {
        if (robotSettings.ENABLE_INDEXER_AUTO_INDEX) {
            indexSensor = new RevDistanceSensor(kOnboard, kInches, kHighAccuracy);
            System.out.println("Enabling index sensor.");
        }
        if (robotSettings.ENABLE_2020_AGITATOR)
            switch (robotSettings.AGITATOR_MOTOR_TYPE) {
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
                    throw new IllegalStateException("No such supported hopper motor config for " + robotSettings.AGITATOR_MOTOR_TYPE.name());
            }
        if (robotSettings.ENABLE_2020_INDEXER)
            switch (robotSettings.INDEXER_MOTOR_TYPE) {
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
                    throw new IllegalStateException("No such supported hopper motor config for " + robotSettings.INDEXER_MOTOR_TYPE.name());
            }
    }

    /*
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
                    leader = new TalonMotorController(robotSettings.SHOOTER_LEADER_ID);
                    if (robotSettings.SHOOTER_USE_TWO_MOTORS) {
                        follower = new TalonMotorController(robotSettings.SHOOTER_FOLLOWER_ID);
                        follower.setSensorToRealDistanceFactor(600 / robotSettings.SHOOTER_SENSOR_UNITS_PER_ROTATION);
                    }
                    leader.setSensorToRealDistanceFactor(600 / robotSettings.SHOOTER_SENSOR_UNITS_PER_ROTATION);
                    break;
                default:
                    throw new IllegalStateException("No such supported shooter motor config for " + robotSettings.SHOOTER_MOTOR_TYPE.name());
            }
    */
    @Override
    public SubsystemStatus getSubsystemStatus() {
        return !indexer.isFailed() && !agitator.isFailed() ? SubsystemStatus.NOMINAL : SubsystemStatus.FAILED;
    }

    @Override
    public void updateTest() {
        if (robotSettings.ENABLE_2020_INDEXER) {
            indexer.moveAtPercent(indexerActive ? 0.9 : 0);
        }
        if (robotSettings.ENABLE_2020_AGITATOR) {
            agitator.moveAtPercent(agitatorActive ? 0.6 : 0);
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
            agitator.moveAtPercent(0);
            indexer.moveAtPercent(0);
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
            return indexSensor.getDistance();
        }
        return -2;
    }

    public boolean isIndexed() {
        return robotSettings.ENABLE_INDEXER_AUTO_INDEX && indexerSensorRange() < robotSettings.INDEXER_DETECTION_CUTOFF_DISTANCE;
    }

    /**
     * Runs every tick. Runs the indexer and agitator motors.
     */
    @Override
    public void updateGeneric() {
        MotorDisconnectedIssue.handleIssue(this, agitator, indexer);
        if (!indexerActive && !agitatorActive) {
            if (robotSettings.ENABLE_2020_INDEXER) {
                if (robotSettings.ENABLE_INDEXER_AUTO_INDEX) {
                    indexer.moveAtPercent(indexerSensorRange() > robotSettings.INDEXER_DETECTION_CUTOFF_DISTANCE ? 0.3 : 0);
                } else {
                    indexer.moveAtPercent(0);
                }
            } //2021 COMP 4 & 2020 COMP 9
            if (robotSettings.ENABLE_2020_AGITATOR) {
                if (robotSettings.ENABLE_INDEXER_AUTO_INDEX) {
                    agitator.moveAtPercent(indexerSensorRange() > robotSettings.INDEXER_DETECTION_CUTOFF_DISTANCE ? 0.5 : 0);
                } else {
                    agitator.moveAtPercent(0);
                }
            }
        } else {
            if (robotSettings.ENABLE_2020_INDEXER) {
                indexer.moveAtPercent(indexerActive ? 0.9 : 0);
            }
            if (robotSettings.ENABLE_2020_AGITATOR) {
                agitator.moveAtPercent(agitatorActive ? 0.6 : 0);
            }
        }
        if (robotSettings.DEBUG && DEBUG) {
            UserInterface.smartDashboardPutBoolean("indexer enable", indexerActive);
            UserInterface.smartDashboardPutBoolean("agitator enable", agitatorActive);
            UserInterface.smartDashboardPutNumber("indexer sensor", indexerSensorRange());
            UserInterface.smartDashboardPutBoolean("hopper2020 indexed", isIndexed());
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
        return "Hopper2020";
    }

    /**
     * applies settings/toggles Agitator and Indexer on/off
     *
     * @param set a boolean to determine wether or not Agitator and Indexer is turned on/off
     */
    public void setAll(boolean set) {
        setAgitator(set);
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
}