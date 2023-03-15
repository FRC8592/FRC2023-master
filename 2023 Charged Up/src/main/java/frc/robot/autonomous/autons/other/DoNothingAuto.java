package frc.robot.autonomous.autons.other;

import frc.robot.autonomous.autons.BaseAuto;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.DelayCommand;

public class DoNothingAuto extends BaseAuto {
    @Override
    public void initialize() {
        queue = new CommandQueue(
            new DelayCommand(1.0)
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
