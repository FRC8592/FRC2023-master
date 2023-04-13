package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class DriveScaler {
    public ShuffleboardTab driveTab = Shuffleboard.getTab("Drive Configuration");
    private SendableChooser<Enum> driveChooser = new SendableChooser<>();

    private double lastOutput;
    private double maxAccel;
    private double maxDecel;

    public enum ScalingMode {
        LINEAR,
        QUADRATIC,
        CUBIC,
        LINEAR_NEWTON_DRIVE, // Not working yet
        QUADRATIC_NEWTON_DRIVE, // Not working yet
    }

    public DriveScaler(double maxAcceleration, double maxDeceleration) {
        driveChooser.setDefaultOption("DEFAULT", ScalingMode.values()[0]);
        for (ScalingMode mode : ScalingMode.values()) {
            driveChooser.addOption(mode.name(), mode);
        }

        driveTab.add("Choose Drive Scaling", driveChooser)
            .withPosition(3, 2)
            .withSize(3, 1);

        maxAccel = maxAcceleration;
        maxDecel = maxDeceleration;
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

    public double applySlew(double input){
        if(Math.signum(input) == Math.signum(lastOutput) || Math.signum(lastOutput) == 0){
            if( input - lastOutput > maxAccel  ){
                    lastOutput += maxAccel;
                    return lastOutput;
            }
            else if(input - lastOutput < -maxAccel ){
                lastOutput -= maxAccel;
                return lastOutput;
            }
        }
        else if( input - lastOutput > maxDecel  ){
            lastOutput += maxDecel;
            return lastOutput;
        }
        else if(input - lastOutput < -maxDecel ){
            lastOutput -= maxDecel;
            return lastOutput;
        }
        lastOutput = input;
        return input;
    }
}
