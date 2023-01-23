package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeCommand extends Command {
    private Timer timer;
    private double forSeconds;
    private IntakeMode mode;
    private boolean isDependent;

    public static enum IntakeMode {
        IN,
        OUT,
    }

    // FOR TESTING
    public IntakeCommand(double forSeconds) {
        this.forSeconds = forSeconds;
    }

    // ACTUAL CONSTRUCTOR
    public IntakeCommand(IntakeMode pMode, double forSeconds) {
        mode = pMode;
        this.forSeconds = forSeconds;
    }

    public IntakeCommand setDependency(boolean dependency) {
        isDependent = dependency;
        return this;
    }

    @Override
    public void initialize(double pTime) {
        timer = new Timer();
        timer.reset();
        timer.start();
    }

    @Override
    public boolean execute() {
        SmartDashboard.putString("Intaking", mode.toString());
        return timer.get() >= forSeconds || !isDependent;
    }

    @Override
    public void shutdown() {
        SmartDashboard.putString("Intaking", IntakeMode.IN.toString());
    }
}
