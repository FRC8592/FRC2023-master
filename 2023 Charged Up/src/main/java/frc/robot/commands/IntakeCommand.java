package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeCommand extends Command {
    private Timer timer;
    private double forSeconds;
    private IntakeMode mode;
    private boolean isDependent;
    private double delay;

    public static enum IntakeMode {
        IN,
        OUT,
    }

    // ACTUAL CONSTRUCTOR
    public IntakeCommand(IntakeMode pMode, double forSeconds) {
        mode = pMode;
        this.forSeconds = forSeconds;
        this.delay = 0;
    }

    //
    // FOR TESTING
    //

    public IntakeCommand(double forSeconds, double delay) {
        this.forSeconds = forSeconds;
        this.delay = delay;
        isDependent = false;
    }

    public IntakeCommand(IntakeMode mode, double forSeconds, double delay) {
        this.mode = mode;
        this.forSeconds = forSeconds;
        this.delay = delay;
        isDependent = false;
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
        if (timer.get() >= delay) {
            SmartDashboard.putString("Intaking", mode.toString());
            return timer.get() >= forSeconds || isDependent;
        }
        return false;
    }

    @Override
    public void shutdown() {
        SmartDashboard.putString("Intaking", IntakeMode.IN.toString());
    }
}
