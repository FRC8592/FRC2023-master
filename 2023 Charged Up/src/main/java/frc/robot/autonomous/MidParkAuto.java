package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Autopark;
import frc.robot.Robot;
import frc.robot.autonomous.trajectory.Trajectories;
import frc.robot.commands.AutobalanceCommand;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.ScoreCommand.Height;
import frc.robot.commands.FollowerCommand;

public class MidParkAuto extends BaseAuto {

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new ScoreCommand(Height.HIGH, scoreTime),
            new FollowerCommand(drive, Trajectories.PARK_B.toTrajectory(), Rotation2d.fromDegrees(180), "Move to Mid"),
            // new FollowerCommand(drive, Trajectories.PARK_TEST.toTrajectory(), Rotation2d.fromDegrees(180), "Move to Mid"),
            new AutobalanceCommand(drive, "AUTO BALANCING")
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
