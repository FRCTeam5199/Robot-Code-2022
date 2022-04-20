package frc.drive.auton.pointtopoint;

/**
 * Used by {@link AutonManager Point to Point} to do cool things with the robot. Processed in {@link
 * AutonManager#updateAuton()}.
 *
 * @author Smaltin
 */
public enum AutonSpecialActions {
    /**
     * You're all good, do nothing
     */
    NONE,
    /**
     * Fire a ball using the shooter
     */
    SHOOT_ONE,
    /**
     * Fire two balls using the shooter
     */
    SHOOT_TWO,
    /**
     * Fire three balls using the shooter
     */
    SHOOT_THREE,
    /**
     * Fire four balls using the shooter
     */
    SHOOT_FOUR,
    /**
     * Fire all (5) balls using the shooter
     */
    SHOOT_ALL,
    /**
     * Runs the shooter for 3 seconds while spun up then stops
     */
    SHOOT_ALL_TIMED,

    /**
     * Pull the intake up using the piston
     */
    INTAKE_UP,
    /**
     * Drop the intake using the piston
     */
    INTAKE_DOWN,
    /**
     * Aim at the target and articulate the hood. Meant for trench.
     */
    AIM_AT_TARGET_TRENCH,


    /**
     * Aim at the target and articulate the hood. Meant for trench, particularly at the trench table
     */
    AIM_AT_TARGET_END_TRENCH,

    /**
     * Sets the turret to its 0 position
     */
    ZERO_TURRET,

    /**
     * Aim at the target and articulate the hood.
     */
    AIM_AT_TARGET_DIRECT,
    /**
     * Spin up the intake to go in
     */
    INTAKE_IN,
    /**
     * Turns off the intake
     */
    INTAKE_OFF,
    /**
     * Resets the hood to 0 and the turret angle to (relatively) 0
     */
    RESET_SHOOTER,

    /**
     * Uses the pitch of the vision camera to rotate the bot towards the target
     */
    AIM_ROBOT_AT_TARGET_PITCH,
    AIM_ROBOT_AT_TARGET_YAW,
    AIM_ROBOT_AT_TARGET_YAW_OFFSET_RIGHT,

    /**
     * Rotates the robot 180 degrees
     */
    DRIVE_180,
    DRIVE_40_REVERSE,
    DRIVE_137_REVERSE,
    DRIVE_165_REVERSE,
    DRIVE_160_REVERSE,
    DRIVE_180_REVERSE,
    DRIVE_130_REVERSE,
    DRIVE_135_REVERSE,
    DRIVE_20_REVERSE,
    DRIVE_3_REVERSE,
    DRIVE_225,
    DRIVE_155,
    DRIVE_70,
    DRIVE_60,
    DRIVE_40,
    DRIVE_10,
    DRIVE_165,
    DRIVE_170,
    DRIVE_172,

    /**
     * Shoots the two balls in the hopper for 2022
     */
    SHOOT_ALL_2022_FURTHER,
    SHOOT_ALL_2022_REAR_BUMPER_ON_TARMAC_LINE,
    SHOOT_ALL_2022_FAR,
    SHOOT_ALL_2022_FAR_FRIAR,
    SHOOT_ALL_2022_VERY_FAR,
    SHOOT_ALL_2022_NOT_FAR_ENOUGH,

    WAIT_ONE,
    DRIVE_BACK_TIMED,
    DRIVE_BACK_TIMED_FRIAR,
    DRIVE_FORWARD_TIMED
}