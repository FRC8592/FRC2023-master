package frc.robot.autonomous;

import frc.robot.autonomous.trajectory.Trajectories;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JointCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.commands.ScoreCommand.Height;

public class TopTwoPieceParkAuto extends BaseAuto {

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new ScoreCommand(Height.HIGH, scoreTime + 1d,"SCORE PRELOAD"),
            new JointCommand(
                new FollowerCommand(drive, Trajectories.TWOPIECE_PARK_A_1.toTrajectory()),
                new IntakeCommand(IntakeMode.OUT, 1d)
            ).tag("DRIVE AND INTAKE"),
            new FollowerCommand(drive, Trajectories.TWOPIECE_PARK_A_2.toTrajectory(), "DRIVE TO SECOND PIECE"),
            new ScoreCommand(Height.HIGH, scoreTime, "SCORE SECOND PIECE"),
            new JointCommand(
                new FollowerCommand(drive, Trajectories.TWOPIECE_PARK_A_3.toTrajectory()),
                new IntakeCommand(IntakeMode.OUT, 1d)
            ).tag("DRIVE AND INTAKE"),
            new FollowerCommand(drive, Trajectories.TWOPIECE_PARK_A_4.toTrajectory(), "PARK ON CHARGING STATION")
        );

        queue.initialize();
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
