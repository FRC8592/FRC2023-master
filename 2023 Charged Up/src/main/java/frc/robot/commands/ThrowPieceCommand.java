package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Intake;

public class ThrowPieceCommand extends Command {
    private Intake intake;
    private Timer timer;
    private double delay = 0;
    private boolean isDependent = false;
    
    public ThrowPieceCommand(Intake intake) {
        this.intake = intake;
        isDependent = true;
    }

    public ThrowPieceCommand(Intake intake, double delay) {
        this.intake = intake;
        this.delay = delay;
        isDependent = true;
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.reset();
        timer.start();
    }

    @Override
    public boolean execute() {
        if (timer.get() >= delay && timer.get() <= 1.5) {
            intake.throwPiece();
            return timer.get() >= 1.5 || isDependent;
        }
        intake.setWrist(0.0);
        intake.stopRoller();
        return isDependent;
    }

    @Override
    public void shutdown() {
        intake.setWrist(0.0);
        intake.stopRoller();
    }
    
}
