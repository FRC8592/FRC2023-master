package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.autonomous.trajectory.Trajectories;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.ScoreCommand.Height;
import frc.robot.commands.FollowerCommand;

public class MidParkAuto extends BaseAuto {

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new ScoreCommand(Height.HIGH, 1.25),
            new FollowerCommand(drive, Trajectories.PARK_B.toTrajectory(), Rotation2d.fromDegrees(0), "Move to Mid")
        );
        
        queue.initialize();
        drive.resetPose(queue.getStartPose());
        if (!Robot.isReal()) {
            Robot.FIELD.setRobotPose(queue.getStartPose());
        }
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
