package frc.pdp;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import frc.misc.*;
import frc.selfdiagnostics.BrownoutIssue;
import frc.selfdiagnostics.UndervoltageIssue;

import static frc.robot.Robot.robotSettings;

/**
 * PowerDistribution (Power Distribution Panel/Module) contains information about power, current, and voltage for the robot. Is cosmetic for
 * now, but should become more useful in the future in diagnosing critical failures
 */
public class PowerDistribution implements ISubsystem {
    /*
    private static final NetworkTableEntry allEnergy = UserInterface.PDP_TOTAL_ENERGY_ON_THIS_BOOT.getEntry(),
            peakCurrent = UserInterface.PDP_PEAK_CURRENT.getEntry(),
    //otherEnergy = UserInterface.PDP_OTHER_ENERGY.getEntry(),
    peakPower = UserInterface.PDP_PEAK_POWER.getEntry();
*/
    private edu.wpi.first.wpilibj.PowerDistribution powerDistributionPanel;
    private final double peakCurrentVal = 0;
    private final double peakPowerVal = 0;
    private final boolean DEBUG = false;

    public PowerDistribution() {
        addToMetaList();
        init();
    }

    @Override
    public void init() {
        powerDistributionPanel = new edu.wpi.first.wpilibj.PowerDistribution(robotSettings.PDP_ID, robotSettings.POWER_DISTRIBUTION_MODULE_TYPE);
        if (robotSettings.POWER_DISTRIBUTION_MODULE_TYPE == ModuleType.kRev) {
            System.out.println("Thank you REV for making me include this shit");
            powerDistributionPanel.clearStickyFaults();
        }
    }

    @Override
    public SubsystemStatus getSubsystemStatus() {
        return RobotController.getBatteryVoltage() > 8 ? SubsystemStatus.NOMINAL : SubsystemStatus.FAILED;
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
        updateGeneric();
    }

    @Override
    public void updateGeneric() {
        if (robotSettings.POWER_DISTRIBUTION_MODULE_TYPE == ModuleType.kCTRE) {
            double BatteryMinVoltage = UserInterface.PDP_BROWNOUT_MIN_OVERRIDE.getEntry().getBoolean(false) ? UserInterface.PDP_BROWNOUT_MIN_VAL.getEntry().getDouble(7) : 7;
            if (DEBUG && robotSettings.DEBUG)
                System.out.println("Read voltage: " + RobotController.getBatteryVoltage() + "V");
            BrownoutIssue.handleIssue(this, RobotController.getBatteryVoltage() < BatteryMinVoltage && RobotController.getBatteryVoltage() > 0);
            UndervoltageIssue.handleIssue(this, RobotController.getBatteryVoltage() >= BatteryMinVoltage && RobotController.getBatteryVoltage() <= (BatteryMinVoltage + 2));
        }
    }

    public void setToggleable(boolean on) {
        if (robotSettings.POWER_DISTRIBUTION_MODULE_TYPE == ModuleType.kRev) {
            powerDistributionPanel.setSwitchableChannel(on);
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
        return "PowerDistribution";
    }
}
