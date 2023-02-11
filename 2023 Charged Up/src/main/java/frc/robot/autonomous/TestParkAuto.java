package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.autonomous.trajectory.Trajectories;
import frc.robot.commands.AutobalanceCommand;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;

public class TestParkAuto extends BaseAuto {
    @Override
    public void initialize() {

        queue = new CommandQueue(
            new FollowerCommand(drive, Trajectories.PARK_B.toTrajectory(), Rotation2d.fromDegrees(180), "Move towards charging station")
            // new AutobalanceCommand(drive, "AUTO BALANCE")
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
