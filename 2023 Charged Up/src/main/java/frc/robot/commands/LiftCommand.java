package frc.robot.commands;

import frc.robot.Lift;
import frc.robot.Lift.Heights;

public class LiftCommand extends Command {
    private Lift lift;
    private Heights height;

    public LiftCommand(Lift lift, Heights height) {
        this.lift = lift;
        this.height = height;
    }

    @Override
    public void initialize() {
        lift.reset();
    }

    @Override
    public boolean execute() {
        lift.setHeight(height);
        return lift.atReference();
    }

    @Override
    public void shutdown() {
        // Nothing yet
    }
    


}
