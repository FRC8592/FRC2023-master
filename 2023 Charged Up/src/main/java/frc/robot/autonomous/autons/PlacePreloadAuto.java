package frc.robot.autonomous.autons;

import frc.robot.Elevator.Heights;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.ScoreCommand;

public class PlacePreloadAuto extends BaseAuto {
    
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        queue = new CommandQueue(
            new LiftCommand(elevator, Heights.HIGH),
            new ScoreCommand(intake),
            new LiftCommand(elevator, Heights.STOWED)
        );
        
    }
    @Override
    public void periodic() {
        queue.run();
    }

}
