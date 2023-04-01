package frc.robot.autonomous.autons.middle;

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
import frc.robot.commands.ThrowPieceCommand;
import frc.robot.commands.PipelineCommand.Pipeline;

import static frc.robot.autonomous.AutonomousPositions.*;

public class MiddleLeftConeGrabCubeBalanceAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);

    private SwerveTrajectory D_TO_Ib = generate(
        config
            .setStartVelocity(0.0)
            .setEndVelocity(1.0)
            .setReversed(false),
        GRID_D.getPose(),
        GRID_D.translate(1.0, -0.1),
        GRID_D.translate(3.0, -0.1)
    );

    private SwerveTrajectory Ib_TO_GP2 = generate(
        config
            .setStartVelocity(1.0)
            .setEndVelocity(0.5)
            .setReversed(false),
        GRID_D.translate(3.0, -0.1),
        GRID_D.translate(4.5, 0.0)
    );

    private SwerveTrajectory GP2_TO_BM = generate(
        config
            .setStartVelocity(0.5)
            .setEndVelocity(1.0)
            .setReversed(true),
        GRID_D.translate(4.5, 0),
        BALANCE_MIDDLE.getPose(),
        BALANCE_MIDDLE.translate(-1.2, 0.0)
    );

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new LiftCommand(elevator, Heights.PRIME), // Tilt up
            new JointCommand( // Lift to high height and score pre-load piece
              new LiftCommand(elevator, Heights.HIGH),
              new ScoreCommand(intake)  
            ),
            new LiftCommand(elevator, Heights.PRIME),
            new JointCommand(
                new FollowerCommand(drive, D_TO_Ib.addRotation(Rotation2d.fromDegrees(180), Math.PI, 0.25)),
                new LiftCommand(elevator, Heights.STOWED),
                new IntakeCommand(intake, 3.0),
                new PipelineCommand(vision, Pipeline.CUBE)
            ),
            new JointCommand(
                new FollowerCommand(drive, vision, Ib_TO_GP2.addRotation(Rotation2d.fromDegrees(180))),
                new IntakeCommand(intake)
            ),
            new JointCommand(
                new FollowerCommand(drive, GP2_TO_BM)
            ),
            new JointCommand(
                new AutobalanceCommand(drive),
                new ThrowPieceCommand(intake)
            )
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
