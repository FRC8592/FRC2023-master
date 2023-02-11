package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Autopark;
import frc.robot.Drivetrain;
import frc.robot.Robot;

public class AutobalanceCommand extends Command {
    private Drivetrain drive;
    private Autopark autopark;
    private Timer timer;

    public AutobalanceCommand(Drivetrain drive) {
        this.drive = drive;
    }

    public AutobalanceCommand(Drivetrain drive, String tag) {
        this.drive = drive;
        setTag(tag);
    }

    @Override
    public void initialize() {
        autopark = new Autopark();
        timer = new Timer();
        timer.reset();
        timer.start();
    }

    @Override
    public boolean execute() {
        if (Robot.isReal()) {
            return autopark.balance(drive);
        } else {
            return timer.get() >= 5d;
        }
    }

    @Override
    public void shutdown() {
        drive.setWheelLock();
    }
}
