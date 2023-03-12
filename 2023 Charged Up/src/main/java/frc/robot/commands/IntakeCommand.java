package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Intake;

public class IntakeCommand extends Command {
    private Intake intake;
    private Timer timer;
    private double delay = 0;
    private boolean isDependent = false;

    public IntakeCommand(Intake intake) {
        this.intake = intake;
    }

    public IntakeCommand(Intake intake, double delay) {
        this.intake = intake;
        this.delay = delay;
    }

    public IntakeCommand(Intake intake, boolean dependency) {
        this.intake = intake;
        isDependent = dependency;
    }

    public IntakeCommand(Intake intake, double delay, boolean dependency) {
        this.intake = intake;
        this.delay = delay;
        isDependent = dependency;
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.reset();
        timer.start();
    }

    @Override
    public boolean execute() {
        if (timer.get() >= delay) {
            intake.enableWrist(true);
            intake.intakeRoller();
            return timer.get() >= 2.0 || isDependent;
            // return intake.hasPiece() || isDependent;
            
        }
        return isDependent;
    }

    @Override
    public void shutdown() {
        intake.enableWrist(false);
        intake.stopRoller();
    }
}
