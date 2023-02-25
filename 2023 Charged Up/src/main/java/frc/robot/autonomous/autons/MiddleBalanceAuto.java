package frc.robot.autonomous.autons;

import frc.robot.commands.AutobalanceCommand;
import frc.robot.commands.CommandQueue;

public class MiddleBalanceAuto extends BaseAuto {

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new AutobalanceCommand(drive)
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
