package frc.robot.autonomous;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.autonomous.blue.BlueCableCoverTestAuto;
import frc.robot.autonomous.blue.BlueCableCoverThreePieceAuto;
import frc.robot.autonomous.blue.BlueCableCoverTwoPieceParkAuto;
import frc.robot.autonomous.blue.BlueCableCoverWaypointAuto;
public class AutonomousSelector {

    public ShuffleboardTab autonTab = Shuffleboard.getTab("Auton Configuration");
    private SendableChooser<Class<?>> autonChooser = new SendableChooser<>();
    private GenericEntry delayEntry;

    public Class<?>[] autos = {
            // MidParkAuto.class,
            // TopThreePieceAuto.class,
            // TopTwoPieceParkAuto.class,
            // WaypointAuto.class,
            // TestParkAuto.class,
            // CoordinateBasedAuto.class,
            BlueCableCoverThreePieceAuto.class,
            BlueCableCoverTwoPieceParkAuto.class,
            BlueCableCoverTestAuto.class,
            BlueCableCoverWaypointAuto.class
    };

    public AutonomousSelector() {
        autonChooser.setDefaultOption("DEFAULT", autos[0]);
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
