package frc.robot.autonomous.autons.other;

import frc.robot.Elevator.Heights;
import frc.robot.autonomous.autons.BaseAuto;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.JointCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.ScoreCommand;

public class PreloadHighMobilityAuto extends BaseAuto {

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new LiftCommand(elevator, Heights.PRIME), // Tilt up
            new JointCommand(
                new ScoreCommand(intake), // Pull out the intake and score the pre-loaded piece
                new LiftCommand(elevator, Heights.HIGH)
            ),
            new LiftCommand(elevator, Heights.STOWED)
        );
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        
    }
    
}
