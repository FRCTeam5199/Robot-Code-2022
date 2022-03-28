package frc.sensors.camera;

import frc.misc.ISubsystem;

import static frc.robot.Robot.robotSettings;

/**
 * I'm just simply vibing here, calm down bro. Anyone that can SEE would use me
 *
 * @author Smaltin
 */
public interface IVision extends ISubsystem {

    boolean hasValidTarget();

    /**
     * Changes the mode of the vision (on, off, blink)
     *
     * @param ledMode the mode (on, off, blink) from enum {@link VisionLEDMode}
     */
    void setLedMode(VisionLEDMode ledMode);

    static IVision manufactureGoalCamera(SupportedVision cameraType) {
        switch (cameraType) {
            case LIMELIGHT:
                return GoalLimelight.GOAL_LIME_LIGHT;
            case PHOTON:
                return GoalPhoton.GOAL_PHOTON;
            default:
                throw new IllegalStateException("You must have a camera type set.");
        }
    }

    /**
     * Returns the angle between the camera and the object
     *
     * @return the angle in degrees between the camera and the object
     */
    default double getAngle() {
        return getAngle(0);
    }

    double getAngle(int targetId);

    /**
     * Returns the pitch between the camera and the object
     *
     * @return the altitude in degrees between the camera and the object
     */
    default double getPitch() {
        return getPitch(0);
    }

    double getPitch(int targetId);

    /**
     * Returns the pitch between the camera and the object, but filtered
     *
     * @return the altitude in degrees after smoothing
     */
    default double getAngleSmoothed() {
        return getAngleSmoothed(0);
    }

    double getAngleSmoothed(int targetId);

    /**
     * Returns the size of the object as visible from the camera
     *
     * @return the size in degrees between the camera and the object
     */
    default double getSize() {
        return getSize(0);
    }

    /**
     * How far are we from the target? Let's find out!
     * Uses the camera's {@link IVision#getPitch() pitch} to find the target offset
     *
     * @return the distance in inches from the target
     */
    default double getDistanceUsingPitch() {
        double angleToGoalDegrees = robotSettings.CAMERA_ANGLE + getPitch();
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
        return (robotSettings.TARGET_HEIGHT - robotSettings.CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);
    }

    /**
     * How far are we from the target? Let's find out!
     * Uses the camera's {@link IVision#getAngle() yaw} to find the target offset
     *
     * @return the distance in inches from the target
     */
    default double getDistanceUsingYaw() {
        double angleToGoalDegrees = robotSettings.CAMERA_ANGLE + getAngle();
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
        return (robotSettings.TARGET_HEIGHT - robotSettings.CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);
    }

    double getSize(int targetId);

    /**
     * A nice place for all the modes of the camera's LEDs ({@link VisionLEDMode#OFF}, {@link VisionLEDMode#ON}, {@link
     * VisionLEDMode#BLINK}).
     */
    enum VisionLEDMode {
        /**
         * Turns off the LEDs
         */
        OFF(),
        /**
         * Blinks the LEDs (dunno why you'd ever do this, but okay..)
         */
        BLINK(),
        /**
         * Turns on the LEDs
         */
        ON()
    }

    /**
     * What dont you get about SIMPLY VIBING?
     */
    enum SupportedVision {
        PHOTON, LIMELIGHT
    }
}
