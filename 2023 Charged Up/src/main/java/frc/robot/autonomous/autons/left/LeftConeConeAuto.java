package frc.robot.autonomous.autons.left;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Elevator.Heights;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.autonomous.autons.BaseAuto;
import frc.robot.commands.AutobalanceCommand;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JointCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.PipelineCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.PipelineCommand.Pipeline;
import static frc.robot.autonomous.AutonomousPositions.*;

public class LeftConeConeAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(5, 3);

    private SwerveTrajectory A_TO_GP1 = generate(
        config,
        GRID_A.getPose(),
        GAME_PIECE_1.getPose()
    ).addRotation(Rotation2d.fromDegrees(180), 0.5);

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new FollowerCommand(drive, vision, A_TO_GP1)
        );
    }

    @Override
    public void periodic() {
        queue.run();        
    }
    
}
