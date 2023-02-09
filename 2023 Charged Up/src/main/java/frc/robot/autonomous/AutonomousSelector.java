package frc.robot.autonomous;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousSelector {

    public ShuffleboardTab autonTab = Shuffleboard.getTab("Auton Configuration");
    private SendableChooser<Class<?>> autonChooser = new SendableChooser<>();
    private SendableChooser<Double> delayChooser = new SendableChooser<>();

    public Class<?>[] autos = {
        MidParkAuto.class,
        TopThreePieceAuto.class,
        TopTwoPieceParkAuto.class,
        WaypointAuto.class,
        SimpleWaypointAuto.class,
        PrecisionTestingWaypointAuto.class
    };

    public AutonomousSelector() {
        autonChooser.setDefaultOption("DEFAULT", autos[0]);
        for (Class<?> auto : autos) {
            autonChooser.addOption(auto.getSimpleName(), auto);
        }

        autonTab.add("Choose Autonomous", autonChooser)
            .withPosition(3, 3)
            .withSize(4, 2);

        delayChooser.setDefaultOption("DEFAULT", 0.0);
        autonTab.add("Choose Delay", delayChooser)
            .withPosition(3, 6)
            .withSize(4, 2);

        SmartDashboard.putNumber("Autonomous Delay", 0d);
    }

    public BaseAuto getSelectedAutonomous() {
        try {
            BaseAuto selected = (BaseAuto) autonChooser.getSelected().getDeclaredConstructor().newInstance();
            SmartDashboard.putString("SELECTED AUTONOMOUS", selected.getClass().getSimpleName());
            return selected;
        } catch (Exception e) {
            return null;
        }
    }
}
