package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

public class DelayCommand extends Command {
    private Timer timer = new Timer();
    private double seconds;

    public DelayCommand(double seconds) {
        this.seconds = seconds;
    }

    public void setDelay(double seconds) {
        this.seconds = seconds;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public boolean execute() {
        timer.start();
        return timer.get() >= seconds;
    }

    @Override
    public void shutdown() {
        timer.reset();
    }
    
}
