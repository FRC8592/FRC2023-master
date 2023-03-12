package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Intake;
import frc.robot.Elevator;
import frc.robot.Robot;
import frc.robot.Elevator.Heights;

public class ScoreCommand extends Command {
    private Timer timer;
    private Intake intake;
    private double delay = 0;

    public ScoreCommand(Intake intake) {
        this.intake = intake;
    }

    public ScoreCommand(Intake intake, double delay) {
        this.intake = intake;
        this.delay = delay;
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.reset();
        timer.start();
    }

    @Override
    public boolean execute() {
        if (Robot.isReal()) {
            // if (timer.get() >= delay) {
            //     intake.enableWrist(true);
            //     intake.scoreRoller();
            //     return !intake.hasPiece();
            // }
            intake.enableWrist(true);
            intake.scoreRoller();
            return timer.get() >= 1.5;
        }
        return timer.get() >= 1.0;
    }

    @Override
    public void shutdown() {
        // Probably don't need a shutdown
        // Might set velocity to 0 but that would be about it
        intake.stopRoller();
        intake.enableWrist(false);
    }
}
