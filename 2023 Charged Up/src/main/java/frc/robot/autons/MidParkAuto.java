package frc.robot.autons;

import frc.robot.autons.trajectory.Trajectories;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.ScoreCommand.Height;
import frc.robot.commands.FollowerCommand;

public class MidParkAuto extends BaseAuto {

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new ScoreCommand(Height.HIGH, 1.25),
            new FollowerCommand(drive, Trajectories.MOBILITY_A.toTrajectory())
        );
        queue.initialize();
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
