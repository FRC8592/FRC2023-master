package frc.robot.autonomous.autons.other;

import frc.robot.Elevator.Heights;
import frc.robot.autonomous.autons.BaseAuto;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.JointCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.ScoreCommand;

public class PreloadAuto extends BaseAuto {

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new LiftCommand(elevator, Heights.PRIME), // PRIME 4-bar
            new JointCommand( // Lift elevator HIGH and score
                new ScoreCommand(intake), 
                new LiftCommand(elevator, Heights.HIGH)
            ),
            new LiftCommand(elevator, Heights.STOWED) // STOW elevator
        );
    }

    @Override
    public void periodic() {
        
    }
    
}
