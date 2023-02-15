package frc.robot.autonomous;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.autonomous.autons.BaseAuto;
import frc.robot.autonomous.autons.CableCoverThreePieceAuto;
import frc.robot.autonomous.autons.CableCoverTwoPieceParkAuto;
import frc.robot.autonomous.autons.LoadingZoneThreePieceAuto;
import frc.robot.autonomous.autons.LoadingZoneTwoPieceParkAuto;
import frc.robot.autonomous.autons.MiddlePreloadParkAuto;
import frc.robot.autonomous.autons.MoveOutCommunityAuto;
import frc.robot.autonomous.autons.WaypointTestingAuto;
public class AutonomousSelector {

    public ShuffleboardTab autonTab = Shuffleboard.getTab("Auton Configuration");
    private SendableChooser<Class<?>> autonChooser = new SendableChooser<>();
    private GenericEntry delayEntry;

    public Class<?>[] autos = {
        CableCoverTwoPieceParkAuto.class,
        CableCoverThreePieceAuto.class,
        LoadingZoneTwoPieceParkAuto.class,
        LoadingZoneThreePieceAuto.class,
        MiddlePreloadParkAuto.class,
        WaypointTestingAuto.class
    };

    public AutonomousSelector() {
        autonChooser.setDefaultOption("DEFAULT", MoveOutCommunityAuto.class);
        for (Class<?> auto : autos) {
            autonChooser.addOption(auto.getSimpleName(), auto);
        }

        autonTab.add("Choose Autonomous", autonChooser)
                .withPosition(3, 3)
                .withSize(4, 2);

        delayEntry = Shuffleboard.getTab("Auton Configuration").add("Autonomous Delay", 0d).getEntry();
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
