package frc.robot.autonomous;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousSelector {

    public ShuffleboardTab autonTab = Shuffleboard.getTab("Auton Configuration");
    private SendableChooser<Class<?>> chooser = new SendableChooser<>();

    public Class<?>[] autos = {
        MidParkAuto.class,
        TopThreePieceAuto.class,
        TopTwoPieceParkAuto.class
    };

    public AutonomousSelector() {
        chooser.setDefaultOption("DEFAULT", autos[0]);
        for (Class<?> auto : autos) {
            chooser.addOption(auto.getSimpleName(), auto);
        }

        autonTab.add("Choose Autonomous", chooser)
            .withPosition(3, 3)
            .withSize(4, 2);
    }

    public BaseAuto getSelectedAutonomous() {
        try {
            BaseAuto selected = (BaseAuto) chooser.getSelected().getDeclaredConstructor().newInstance();
            SmartDashboard.putString("SELECTED AUTONOMOUS", selected.getClass().getSimpleName());
            return selected;
        } catch (Exception e) {
            return null;
        }
    }
}
