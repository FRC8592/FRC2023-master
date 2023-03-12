package frc.robot.autonomous;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.autonomous.autons.BaseAuto;
import frc.robot.autonomous.autons.MoveOutCommunityAuto;
import frc.robot.autonomous.autons.cablecover.CableCoverPreloadBalanceAuto;
import frc.robot.autonomous.autons.cablecover.CableCoverPreloadMobilityAuto;
import frc.robot.autonomous.autons.cablecover.CableCoverTwoPieceAuto;
import frc.robot.autonomous.autons.loadingzone.LoadingZonePreloadBalanceAuto;
import frc.robot.autonomous.autons.loadingzone.LoadingZonePreloadMobilityAuto;
import frc.robot.autonomous.autons.loadingzone.LoadingZoneTwoPieceAuto;
import frc.robot.autonomous.autons.middle.MiddlePreloadBalanceAuto;
import frc.robot.autonomous.autons.middle.MiddleBalanceAuto;
public class AutonomousSelector {

    public ShuffleboardTab autonTab = Shuffleboard.getTab("Auton Configuration");
    private SendableChooser<Class<?>> autonChooser = new SendableChooser<>();
    private GenericEntry delayEntry;

    public Class<?>[] autos = {
        MoveOutCommunityAuto.class, // Ready to test
        MiddlePreloadBalanceAuto.class, // Ready to test
        MiddleBalanceAuto.class,
        LoadingZonePreloadMobilityAuto.class, // Ready to test
        LoadingZonePreloadBalanceAuto.class, // Ready to test
        LoadingZoneTwoPieceAuto.class,
        CableCoverPreloadMobilityAuto.class, // Ready to test
        CableCoverPreloadBalanceAuto.class, // Ready to test
        CableCoverTwoPieceAuto.class
    };

    public AutonomousSelector() {
        autonChooser.setDefaultOption("DEFAULT", MoveOutCommunityAuto.class);
        for (Class<?> auto : autos) {
            autonChooser.addOption(auto.getSimpleName(), auto);
        }

        autonTab.add("Choose Autonomous", autonChooser)
            .withPosition(3, 2)
            .withSize(4, 2);

        delayEntry = autonTab.add("Autonomous Delay", 0d)
            .withPosition(4, 1)
            .withSize(2, 1)
            .getEntry();
    }

    public double getDelay() {
        return delayEntry.getDouble(0.0);
    }

    public BaseAuto getSelectedAutonomous() {
        try {
            BaseAuto selected = (BaseAuto) autonChooser.getSelected().getDeclaredConstructor().newInstance();
            return selected;
        } catch (Exception e) {
            return null;
        }
    }
}
