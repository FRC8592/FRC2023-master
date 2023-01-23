package frc.robot.autons;

import frc.robot.Robot;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JointCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.commands.ScoreCommand.Height;

import static frc.robot.autons.trajectory.Trajectories.*;

public class TopThreePieceAuto extends BaseAuto {
    
    @Override
    public void initialize() {
        queue = new CommandQueue(
            new ScoreCommand(Height.HIGH, 1.25),
            new JointCommand(
                new FollowerCommand(drive, THREEPIECE_A_1.toTrajectory()),
                new IntakeCommand(IntakeMode.OUT, 2).setDependency(false)
            ),
            new FollowerCommand(drive, THREEPIECE_A_2.toTrajectory()),
            new ScoreCommand(Height.HIGH, 1.25),
            new JointCommand(
                new FollowerCommand(drive, THREEPIECE_A_3.toTrajectory()),
                new IntakeCommand(IntakeMode.OUT, 2).setDependency(false)
            ),
            new FollowerCommand(drive, THREEPIECE_A_4.toTrajectory()),
            new ScoreCommand(Height.MID, 1.25)
        );

        queue.initialize();
    }

    @Override
    public void periodic() {
        queue.run();
    }
}
