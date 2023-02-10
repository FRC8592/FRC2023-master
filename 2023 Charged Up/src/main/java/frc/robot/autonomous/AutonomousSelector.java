package frc.robot.autonomous;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousSelector {

    public ShuffleboardTab autonTab = Shuffleboard.getTab("Auton Configuration");
    private SendableChooser<Class<?>> autonChooser = new SendableChooser<>();
    ShuffleboardContainer container;

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
            .withSize(4, 2)
        ;
        
        // container = autonTab.getLayout("Auto Data", BuiltInLayouts.kGrid)
        // .withSize(2, 4)
        // .withPosition(0, 0);

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
