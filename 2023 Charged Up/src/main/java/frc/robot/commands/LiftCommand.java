package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Elevator;
import frc.robot.Robot;
import frc.robot.Elevator.Heights;

public class LiftCommand extends Command {
    private Timer timer;
    private Elevator lift;
    private Heights height;
    private double delay = 0;

    public LiftCommand(Elevator lift, Heights height) {
        this.lift = lift;
        this.height = height;
    }

    public LiftCommand(Elevator lift, Heights height, double delay) {
        this.lift = lift;
        this.height = height;
        this.delay = delay;
    }

    @Override
    public void initialize() {
        // lift.reset();
        timer = new Timer();
        timer.reset();
        timer.start();
    }

    @Override
    public boolean execute() {
        if (Robot.isReal()) {
            if (timer.get() >= delay) {
                lift.set(height);
                return lift.atReference();
            }
            return false;
        }
        return timer.get() >= 1.0;
    }

    @Override
    public void shutdown() {
        // lift.testPlanTilt(null);
        // lift.testPlanLift(null);
        // lift.set(Heights.STALL);
    }
    


}
