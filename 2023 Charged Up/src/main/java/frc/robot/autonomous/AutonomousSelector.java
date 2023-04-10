package frc.robot.autonomous;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.autonomous.autons.*;
import frc.robot.autonomous.autons.left.*;
import frc.robot.autonomous.autons.middle.*;
import frc.robot.autonomous.autons.other.*;
import frc.robot.autonomous.autons.right.*;

public class AutonomousSelector {
    public ShuffleboardTab autonTab = Shuffleboard.getTab("Auton Configuration");
    private SendableChooser<Class<?>> autonChooser = new SendableChooser<>();
    private GenericEntry delayEntry;

    public Class<?>[] autos = {
        MoveOutCommunityAuto.class,
        PreloadAuto.class,

        LeftConeBalanceAuto.class,
        LeftConeGrabCubeBalanceAuto.class,
        LeftConeCubeAuto.class,
        LeftConeCubeBalanceAuto.class,
        LeftConeCubeGrabConeAuto.class,
        LeftCableCoverConeCubeAuto.class,

        MiddleConeBalanceAuto.class,
        MiddleLeftConeGrabCubeBalanceAuto.class,
        MiddleRightConeGrabCubeBalanceAuto.class,
        
        RightConeBalanceAuto.class,
        RightConeGrabCubeBalanceAuto.class,
        RightConeCubeAuto.class,
        RightConeConeAuto.class,
        RightConeCubeBalanceAuto.class,
        RightCableCoverConeCubeAuto.class,
        // RightCableCoverConeCubeGrabCubeAuto.class
    };

    public AutonomousSelector() {
        autonChooser.setDefaultOption("DEFAULT - DO NOTHING", LeftConeCubeCubeAuto.class);
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
