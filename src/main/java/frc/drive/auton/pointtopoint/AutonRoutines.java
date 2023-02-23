package frc.drive.auton.pointtopoint;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.drive.auton.Point;

import java.util.*;

import static frc.drive.auton.pointtopoint.AutonSpecialActions.*;

/**
 * All the auton paths, using {@link Point points} and {@link AutonSpecialActions} to make the robot do the do
 */
public enum AutonRoutines {
    DO_NOTHING(
            new AutonWaypoint(new Point(0, 0))
    ),
    /*
    RESET_INTAKE_DO_NOTHING(
            new AutonWaypoint(new Point(0, 0), INTAKE_UP)
    ),
     */
    DRIVE_OFF_INIT_LINE(
            new AutonWaypoint(new Point(0, 0)),
            new AutonWaypoint(new Point(1.25, 0))
    ),
    GAMER_TEST(
        new AutonWaypoint(new Point(0, 0)),
        new AutonWaypoint(new Point(1, 0)),
        new AutonWaypoint(new Point(0, 0))
    ),
    TEST(
            new AutonWaypoint(new Point(8999, 0))
    ),
    ONE_BALL_AUTON(
            new AutonWaypoint(new Point(0,0), 1),
            new AutonWaypoint(new Point(1.4 , 0), 1), //(1.7741787849766635, 0.01580735327687336)
            new AutonWaypoint(DRIVE_165_REVERSE),
            new AutonWaypoint(AIM_ROBOT_AT_TARGET_YAW),
            new AutonWaypoint(WAIT_ONE),
            new AutonWaypoint(SHOOT_ALL_2022_REAR_BUMPER_ON_TARMAC_LINE),
            new AutonWaypoint(AIM_ROBOT_AT_TARGET_YAW_STOP)
    ),
    SHOOT_WHILE_STILL_2022(
            new AutonWaypoint(new Point(0, 0), SHOOT_ALL_2022_FURTHER)
    ),
    /*
    DRIVE_OFF_TARMAC_BACKWARDS(
            new AutonWaypoint(new Point(0, 0)),
            new AutonWaypoint(new Point(-1.5, 0), -1)
    ),
    AIM_ROBOT_AT_TARGET(
            new AutonWaypoint(new Point(0, 0), AIM_ROBOT_AT_TARGET_PITCH)
    ),
    ROTATE_180_DRIVE_FORWARD(
            new AutonWaypoint(new Point(0, 0), DRIVE_180),
            new AutonWaypoint(new Point(-1, 0))
    ),
     */
    PICKUP_ONE_SHOOT_TWO_2022(
            new AutonWaypoint(new Point(0, 0), INTAKE_DOWN),
            new AutonWaypoint(new Point(0, 0), 1, INTAKE_IN),
            new AutonWaypoint(new Point(1.9, 0), 1), //(1.7741787849766635, 0.01580735327687336)
            new AutonWaypoint(new Point(1.9, 0), 1, DRIVE_180),
            new AutonWaypoint(AIM_ROBOT_AT_TARGET_PITCH),
            new AutonWaypoint(SHOOT_ALL_2022_FAR)
            //new AutonWaypoint(new Point(0.05, 0), AIM_ROBOT_AT_TARGET_PITCH),
            //new AutonWaypoint(new Point(0.05, 0), SHOOT_ALL_2022_BEHIND_TARMAC),
            //new AutonWaypoint(new Point(-1.9, 0), -1)
    ),

    PICKUP_ONE_DRIVE_SHOOT_TWO_2022(
            new AutonWaypoint(new Point(0, 0), 1, INTAKE_IN),
            new AutonWaypoint(new Point(2.2, 0), 1), //(1.7741787849766635, 0.01580735327687336)
            new AutonWaypoint(DRIVE_155),
            new AutonWaypoint(AIM_ROBOT_AT_TARGET_PITCH),
            new AutonWaypoint(SHOOT_ALL_2022_FAR)
    ),

    TWO_BALL_AUTON(
            new AutonWaypoint(new Point(0, 0), INTAKE_DOWN),
            new AutonWaypoint(new Point(0, 0), 1, INTAKE_IN),
            new AutonWaypoint(new Point(2, 0), 1), //(1.7741787849766635, 0.01580735327687336)
            new AutonWaypoint(DRIVE_160_REVERSE),
            new AutonWaypoint(AIM_ROBOT_AT_TARGET_YAW),
            new AutonWaypoint(SHOOT_ALL_2022_FURTHER),
            new AutonWaypoint(AIM_ROBOT_AT_TARGET_YAW_STOP)
            ),
    FOUR_BALL_AUTON(
            new AutonWaypoint(new Point(0, 0), INTAKE_DOWN ),
            new AutonWaypoint(new Point(0, 0), 1, INTAKE_IN),
            new AutonWaypoint(new Point(2, 0), 1.01), //(1.7741787849766635, 0.01580735327687336)
            new AutonWaypoint(DRIVE_130_REVERSE),
            new AutonWaypoint(AIM_ROBOT_AT_TARGET_YAW),
            new AutonWaypoint(SHOOT_ALL_2022_FAR),
            new AutonWaypoint(AIM_ROBOT_AT_TARGET_YAW_STOP),
            new AutonWaypoint(DRIVE_60),
            new AutonWaypoint(new Point(7.05, -1.6), 1.55),
            new AutonWaypoint(WAIT_ONE),
            new AutonWaypoint(DRIVE_180_REVERSE),
            new AutonWaypoint(AIM_ROBOT_AT_TARGET_YAW),
            new AutonWaypoint(WAIT_ONE),
            new AutonWaypoint(SHOOT_ALL_2022_VERY_FAR),
            new AutonWaypoint(AIM_ROBOT_AT_TARGET_YAW_STOP)
    ),
    FOUR_BALL_AUTON_UGH(
            new AutonWaypoint(new Point(0, 0), INTAKE_DOWN ),
            new AutonWaypoint(new Point(0, 0), 1, INTAKE_IN),
            new AutonWaypoint(new Point(2, 0), 1.01), //(1.7741787849766635, 0.01580735327687336)
            new AutonWaypoint(DRIVE_130_REVERSE),
            new AutonWaypoint(AIM_ROBOT_AT_TARGET_YAW),
            new AutonWaypoint(SHOOT_ALL_2022_FAR),
            new AutonWaypoint(DRIVE_60),
            new AutonWaypoint(new Point(7.2, -1.6), 1.55),
            new AutonWaypoint(DRIVE_180_REVERSE),
            new AutonWaypoint(DRIVE_FORWARD_TIMED),
            new AutonWaypoint(AIM_ROBOT_AT_TARGET_YAW),
            new AutonWaypoint(SHOOT_ALL_2022_NOT_FAR_ENOUGH)
    ),
    FOUR_BALL_AUTON_FRIAR(
            new AutonWaypoint(new Point(0, 0), INTAKE_DOWN),
            new AutonWaypoint(new Point(0, 0), 1, INTAKE_IN),
            new AutonWaypoint(new Point(2, 0), 1), //(1.7741787849766635, 0.01580735327687336)
            new AutonWaypoint(DRIVE_130_REVERSE),
            new AutonWaypoint(AIM_ROBOT_AT_TARGET_YAW),
            new AutonWaypoint(SHOOT_ALL_2022_FAR_FRIAR),
            new AutonWaypoint(DRIVE_155),
            new AutonWaypoint(new Point(7.1, -1.8), 1.77), //7.2 -1.35
            new AutonWaypoint(DRIVE_BACK_TIMED_FRIAR),
            new AutonWaypoint(DRIVE_135_REVERSE),
            //new AutonWaypoint(DRIVE_225),
            new AutonWaypoint(AIM_ROBOT_AT_TARGET_YAW),
            new AutonWaypoint(SHOOT_ALL_2022_VERY_FAR)
    ),
    /*
    PICKUP_ONE_DRIVE_SHOOT_TWO_DRIVE_2022(
            new AutonWaypoint(new Point(0, 0), 1, INTAKE_IN),
            new AutonWaypoint(new Point(1.9, 0), 1), //(1.7741787849766635, 0.01580735327687336)
            new AutonWaypoint(new Point(0.5, 0), -1, DRIVE_180),
            new AutonWaypoint(INTAKE_OFF),
            new AutonWaypoint(AIM_ROBOT_AT_TARGET_PITCH),
            new AutonWaypoint(SHOOT_ALL_2022_REAR_BUMPER_ON_TARMAC_LINE),
            new AutonWaypoint(DRIVE_180),
            new AutonWaypoint(new Point(1.9, 0), 1)
    ),
     */
    SHOOT_ALL_THEN_BACKUP(
            new AutonWaypoint(new Point(0, 0), -1, SHOOT_ALL_2022_REAR_BUMPER_ON_TARMAC_LINE),
            new AutonWaypoint(new Point(-2, 0), -1, NONE)
    ),
    /*
    DRIVE_FORWARD_ROTATE_DRIVE_BACK(
            new AutonWaypoint(new Point(0, 0)),
            new AutonWaypoint(new Point(3, 0), DRIVE_180),
            new AutonWaypoint(new Point(0, 0))
    ),
    ROTATE(
      new AutonWaypoint(new Point(0, 0), DRIVE_180)
    ),
    FORWARD_N_BACK(
            new AutonWaypoint(new Point(0, 0)),
            new AutonWaypoint(new Point(3, 0.01)),
            new AutonWaypoint(new Point(0, 0))
    ),
    DRIVE_OFF_SHOOT_3(
            new AutonWaypoint(new Point(0, 0)),
            new AutonWaypoint(new Point(1.25, 0), AIM_AT_TARGET_TRENCH),
            new AutonWaypoint(new Point(1.25, 0), SHOOT_THREE)
    ),
    SHOOT_3_DRIVE_OFF_INIT_LINE(
            new AutonWaypoint(new Point(0, 0), AIM_AT_TARGET_TRENCH),
            new AutonWaypoint(new Point(0, 0), SHOOT_THREE),
            //new AutonWaypoint(new Point(0, 0), RESET_SHOOTER),
            new AutonWaypoint(new Point(1.25, 0)) //Drive off init line
    ),

    CREATING_AUTON_TESTING(
            new AutonWaypoint(new Point(0, 0), AIM_AT_TARGET_TRENCH),
            new AutonWaypoint(new Point(0, 0), SHOOT_ALL_TIMED),
            new AutonWaypoint(new Point(0, 0), INTAKE_DOWN),
            new AutonWaypoint(new Point(0, 0), INTAKE_IN),
            new AutonWaypoint(new Point(4.5, 0), ZERO_TURRET),
            new AutonWaypoint(new Point(4.5, 0), AIM_AT_TARGET_END_TRENCH),
            new AutonWaypoint(new Point(4.5, 0), INTAKE_OFF),
            new AutonWaypoint(new Point(4.5, 0), SHOOT_ALL_TIMED)
    ),
    SHOOT_3_INTAKE_TRENCH(
            new AutonWaypoint(new Point(0, 0), AIM_AT_TARGET_TRENCH),
            new AutonWaypoint(new Point(0, 0), SHOOT_THREE),
            new AutonWaypoint(new Point(0, 0), INTAKE_DOWN),
            new AutonWaypoint(new Point(0, 0), INTAKE_IN),
            new AutonWaypoint(new Point(4.8, 0), INTAKE_OFF)
    ),
    SHOOT_3_DONT_MOVE(
            new AutonWaypoint(new Point(0, 0), AIM_AT_TARGET_TRENCH),
            new AutonWaypoint(new Point(0, 0), SHOOT_THREE)
    ),
    SHOOT_3_INTAKE_TRENCH_SHOOT_3(
            new AutonWaypoint(new Point(0, 0), AIM_AT_TARGET_TRENCH),
            new AutonWaypoint(new Point(0, 0), SHOOT_THREE),
            new AutonWaypoint(new Point(0, 0), INTAKE_DOWN),
            new AutonWaypoint(new Point(0, 0), INTAKE_IN),
            new AutonWaypoint(new Point(4.5, 0), ZERO_TURRET),
            new AutonWaypoint(new Point(4.5, 0), AIM_AT_TARGET_END_TRENCH),
            new AutonWaypoint(new Point(4.5, 0), INTAKE_OFF),
            new AutonWaypoint(new Point(4.5, 0), SHOOT_THREE)
    ),

    AUTON_TUNING(
            new AutonWaypoint(new Point(0, 0), INTAKE_UP),
            new AutonWaypoint(new Point(Units.feetToMeters(8), 0), INTAKE_DOWN),
            new AutonWaypoint(new Point(Units.feetToMeters(8), 0), INTAKE_IN),
            new AutonWaypoint(new Point(0, -0.3), INTAKE_OFF),
            new AutonWaypoint(new Point(0, -0.3), INTAKE_UP)
    ),
    GO_FORWARD_AND_SHOOT_ONE(
            new AutonWaypoint(new Point(0, 0)),
            new AutonWaypoint(new Point(3, 0), AIM_AT_TARGET_TRENCH),
            new AutonWaypoint(new Point(3, 0), SHOOT_ONE)
    ),
    MULTI_SHOT_TEST(
            //new AutonWaypoint(new Point(0, 0), INTAKE_DOWN),
            new AutonWaypoint((new Point(0, 0))), //do nothing?
            new AutonWaypoint(new Point(0, 0), AIM_AT_TARGET_TRENCH),
            new AutonWaypoint(new Point(0, 0), SHOOT_TWO)
            //new AutonWaypoint(new Point(0, 0), INTAKE_UP)
    ),
    DRIVE_FORWARD_THEN_SHOOT_TWO(
            new AutonWaypoint(new Point(0, 0), INTAKE_DOWN),
            new AutonWaypoint(new Point(0, 0), INTAKE_IN),
            new AutonWaypoint(new Point(4, 0.5), INTAKE_OFF),
            new AutonWaypoint(new Point(4, 0.5), INTAKE_UP),
            new AutonWaypoint(new Point(4, 0.5), AIM_AT_TARGET_TRENCH),
            new AutonWaypoint(new Point(4, 0.5), SHOOT_TWO)
    ),
    GO_FORWARD_GO_BACK(
            new AutonWaypoint(new Point(0, 0), 1),
            new AutonWaypoint(new Point(3, 0), 1),
            new AutonWaypoint(new Point(3, -3), 1),
            new AutonWaypoint(new Point(0, 0), 1)
    ),
    BARREL_RACING_PATH_ONE( //Doesn't work yet, but the numbers are here. -Sterling
            new AutonWaypoint(new Point(Units.feetToMeters(4.084849326482433), Units.feetToMeters(-8.00097208721011)), 1), //Starting Position
            new AutonWaypoint(new Point(Units.feetToMeters(10.625607554506319), Units.feetToMeters(-7.042771837244827)), 1), //Approach Loop 1
            new AutonWaypoint(new Point(Units.feetToMeters(14.125121510901264), Units.feetToMeters(-9.917372587140676)), 1), //Start Loop 1
            new AutonWaypoint(new Point(Units.feetToMeters(11.04221635883905), Units.feetToMeters(-11.292181641438688)), 1), //Mid Loop 1
            new AutonWaypoint(new Point(Units.feetToMeters(12.125399250104152), Units.feetToMeters(-8.500902652409389)), 1), //End Loop 1
            new AutonWaypoint(new Point(Units.feetToMeters(17.957922510762394), Units.feetToMeters(-7.501041522010833)), 1),
            new AutonWaypoint(new Point(Units.feetToMeters(21.45743646715734), Units.feetToMeters(-6.042910706846271)), 1), //Start Loop 2
            new AutonWaypoint(new Point(Units.feetToMeters(21.290792945424247), Units.feetToMeters(-3.501597000416609)), 1), //Mid Loop 2
            new AutonWaypoint(new Point(Units.feetToMeters(18.332870434661853), Units.feetToMeters(-5.75128454381336)), 1), //End Loop 2
            new AutonWaypoint(new Point(Units.feetToMeters(22.37397583668935), Units.feetToMeters(-10.750590195806138)), 1), //Start Loop 3
            new AutonWaypoint(new Point(Units.feetToMeters(26.49840299958339), Units.feetToMeters(-11.000555478405778)), 1), //LowerMid Loop 3
            new AutonWaypoint(new Point(Units.feetToMeters(26.62338564088321), Units.feetToMeters(-8.209276489376476)), 1), //UpperMid Loop 3
            new AutonWaypoint(new Point(Units.feetToMeters(22.54061935842244), Units.feetToMeters(-7.667685043743925)), 1), //End Loop 3
            new AutonWaypoint(new Point(Units.feetToMeters(17.416331065129842), Units.feetToMeters(-6.959450076378282)), 1),
            new AutonWaypoint(new Point(Units.feetToMeters(11.54214692403833), Units.feetToMeters(-6.001249826412999)), 1),
            new AutonWaypoint(new Point(Units.feetToMeters(4.084849326482433), Units.feetToMeters(-6.0845715872795445)), 1) //Ending Position
    ),
    BARREL_RACING_PATH_LOL( //Doesn't work yet, but the numbers are here. -Sterling
            new AutonWaypoint(new Point(Units.feetToMeters(4.084849326482433), Units.feetToMeters(-8.00097208721011)), 1), //Starting Position
            new AutonWaypoint(new Point(Units.feetToMeters(10.625607554506319), Units.feetToMeters(-7.042771837244827)), 1), //Approach Loop 1
            new AutonWaypoint(new Point(Units.feetToMeters(14.125121510901264), Units.feetToMeters(-9.917372587140676)), 1), //Start Loop 1
            new AutonWaypoint(new Point(Units.feetToMeters(11.04221635883905), Units.feetToMeters(-11.292181641438688)), 1), //Mid Loop 1
            new AutonWaypoint(new Point(Units.feetToMeters(12.125399250104152), Units.feetToMeters(-8.500902652409389)), 1), //End Loop 1
            new AutonWaypoint(new Point(Units.feetToMeters(17.957922510762394), Units.feetToMeters(-7.501041522010833)), 1),
            new AutonWaypoint(new Point(Units.feetToMeters(21.45743646715734), Units.feetToMeters(-6.042910706846271)), 1), //Start Loop 2
            new AutonWaypoint(new Point(Units.feetToMeters(21.290792945424247), Units.feetToMeters(-3.501597000416609)), 1), //Mid Loop 2
            new AutonWaypoint(new Point(Units.feetToMeters(18.332870434661853), Units.feetToMeters(-5.75128454381336)), 1), //End Loop 2
            new AutonWaypoint(new Point(Units.feetToMeters(22.37397583668935), Units.feetToMeters(-10.750590195806138)), 1), //Start Loop 3
            new AutonWaypoint(new Point(Units.feetToMeters(26.49840299958339), Units.feetToMeters(-11.000555478405778)), 1), //LowerMid Loop 3
            new AutonWaypoint(new Point(Units.feetToMeters(26.62338564088321), Units.feetToMeters(-8.209276489376476)), 1), //UpperMid Loop 3
            new AutonWaypoint(new Point(Units.feetToMeters(22.54061935842244), Units.feetToMeters(-7.667685043743925)), 1), //End Loop 3
            new AutonWaypoint(new Point(Units.feetToMeters(17.416331065129842), Units.feetToMeters(-6.959450076378282)), 1),
            new AutonWaypoint(new Point(Units.feetToMeters(11.54214692403833), Units.feetToMeters(-6.001249826412999)), 1),
            new AutonWaypoint(new Point(Units.feetToMeters(4.084849326482433), Units.feetToMeters(-6.0845715872795445)), 1) //Ending Position
    ),
    TRACE_CARPET_1_31_2021(
            new AutonWaypoint(new Point(0, 0), 1),
            new AutonWaypoint(new Point(7.494228280781437, 0), 1),
            new AutonWaypoint(new Point(7.494228280781437, -3.6531160166183034), 1),
            new AutonWaypoint(new Point(0, -3.6531160166183034), 1),
            new AutonWaypoint(new Point(0, 0), 1)
    ),

    CARPET_TEST_SLALOM(
            new AutonWaypoint(new Point(0.4858349134944604, -0.8050449256126393), 3),
            new AutonWaypoint(new Point(2.2549439188290363, -0.8585070109386842), 3),
            new AutonWaypoint(new Point(2.264664297979227, -2.447789001994746), 3),
            new AutonWaypoint(new Point(3.926849132661714, -2.4429288124196513), 3),
            new AutonWaypoint(new Point(3.9025481847862387, -0.8487866317884941), 3),
            new AutonWaypoint(new Point(5.603614536069486, -0.8244856839130191), 3),
            new AutonWaypoint(new Point(5.623055294369865, -2.4332084332694612), 3),
            new AutonWaypoint(new Point(3.926849132661714, -2.438068622844556), 3),
            new AutonWaypoint(new Point(3.9171287535115233, -0.8439264422133992), 3),
            new AutonWaypoint(new Point(2.2549439188290363, -0.8536468213635892), 3),
            new AutonWaypoint(new Point(2.264664297979227, -2.447789001994746), 3),
            new AutonWaypoint(new Point(0.5392969988205053, -2.423488054119271), 3)
    )*/;

    private static SendableChooser<AutonRoutines> myChooser;
    public final ArrayList<AutonWaypoint> WAYPOINTS = new ArrayList<>();
    public int currentWaypoint = 0;

    public static SendableChooser<AutonRoutines> getSendableChooser() {
        return Objects.requireNonNullElseGet(myChooser, () -> {
            myChooser = new SendableChooser<>();
            for (AutonRoutines routine : AutonRoutines.values())
                myChooser.addOption(routine.name(), routine);
            return myChooser;
        });
    }

    AutonRoutines(AutonWaypoint... waypoints) {
        WAYPOINTS.addAll(Arrays.asList(waypoints));
    }
}