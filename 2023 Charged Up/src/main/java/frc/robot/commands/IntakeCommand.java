package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeCommand extends Command {
    private Timer timer;

    public static enum IntakeMode {
        IN,
        OUT,
    }

    private double forSeconds = 0;
    private double delay = 0;
    private boolean isDependent = false;
    private IntakeMode mode = IntakeMode.IN;

    // ACTUAL CONSTRUCTOR
    public IntakeCommand(IntakeMode mode, double delay) {
        this.mode = mode;
        this.delay = delay;
        isDependent = true;
    }

    public IntakeCommand(IntakeMode mode, double delay, String tag) {
        this.mode = mode;
        this.delay = delay;
        isDependent = true;
        setTag(tag);
    }

    public IntakeCommand(IntakeMode mode) {
        this.mode = mode;
        isDependent = true;
    }

    // ===============================
    // CONSTRUCTORS FOR TESTING
    // ===============================

    public IntakeCommand(double forSeconds, double delay) {
        this.forSeconds = forSeconds;
        this.delay = delay;
    }

    public IntakeCommand(IntakeMode mode, double forSeconds, double delay) {
        this.mode = mode;
        this.forSeconds = forSeconds;
        this.delay = delay;
    }

    public IntakeCommand(IntakeMode mode, double delay, boolean dependency) {
        this.mode = mode;
        this.delay = delay;
        isDependent = dependency;
    }

    public IntakeCommand setDelay(double delay) {
        this.delay = delay;
        return this;
    }

    public IntakeCommand setDependency(boolean dependency) {
        isDependent = dependency;
        return this;
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.reset();
        timer.start();
        // Initialize intake module
    }

    @Override
    public boolean execute() {
        if (timer.get() >= delay) {
            SmartDashboard.putString("Intaking", mode.toString());
            // Run intake / Open intake claw until game piece in range
            return timer.get() >= forSeconds || isDependent;
        }
        return false;
    }

    @Override
    public void shutdown() {
        SmartDashboard.putString("Intaking", IntakeMode.IN.toString());
    }
}
