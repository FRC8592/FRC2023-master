package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ScoreCommand extends Command {
    private Timer timer;
    private double forSeconds;
    private Height height;

    public static enum Height {
        OFF,
        LOW,
        MID,
        HIGH
    }

    // FOR TESTING
    public ScoreCommand(Height pHeight, double forSeconds) {
        height = pHeight;
        this.forSeconds = forSeconds;
    }

    // ACTUAL CONSTRUCTOR
    public ScoreCommand(Height pHeight) {
        height = pHeight;
    }

    @Override
    public void initialize(double pTime) {
        timer = new Timer();
        timer.reset();
        timer.start();
    }

    @Override
    public boolean execute() {
        SmartDashboard.putString("Lifting to height", height.toString());
        return timer.get() >= forSeconds;
    }

    @Override
    public void shutdown() {
        SmartDashboard.putString("Lifting to height", Height.OFF.toString());
    }
}
