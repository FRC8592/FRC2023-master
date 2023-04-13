package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class DriveScaler {
    public ShuffleboardTab driveTab = Shuffleboard.getTab("Drive Configuration");
    private SendableChooser<Enum> driveChooser = new SendableChooser<>();

    public enum ScalingMode {
        LINEAR,
        QUADRATIC,
        CUBIC,
        LINEAR_NEWTON_DRIVE, // Not working yet
        QUADRATIC_NEWTON_DRIVE, // Not working yet
    }

    public DriveScaler() {
        driveChooser.setDefaultOption("DEFAULT", ScalingMode.values()[0]);
        for (ScalingMode mode : ScalingMode.values()) {
            driveChooser.addOption(mode.name(), mode);
        }

        driveTab.add("Choose Drive Scaling", driveChooser)
            .withPosition(3, 2)
            .withSize(3, 1);
    }

    public double scale(double value) {
        double deadband = ConfigRun.JOYSTICK_DEADBAND;
        switch(driveChooser.getSelected().name()) {
            case "LINEAR":
                return value;
            case "QUADRATIC":
                return Math.pow(value, 2) * Math.signum(value);
            case "CUBIC":
                return Math.pow(value, 3);
            case "NEWTON_DRIVE":
                return value; // Come up with a cool, unique scaling method
            case "LINEAR_NEWTON_DRIVE":
                return (1/(1-deadband))*(value-deadband);
            case "QUADRATIC_NEWTON_DRIVE":
                return Math.pow((1/(1-deadband))*(value-deadband), 2);
            default:
                return value;
        }
    }

    public double slewFilter(double currentVelo, double setPoint, double maxAccel){
        double delta = setPoint - currentVelo;
        
        delta = Math.signum(delta) * Math.min(maxAccel, Math.abs(delta));

        double calculatedResult = currentVelo + delta;

  

        return calculatedResult;
    }
}
