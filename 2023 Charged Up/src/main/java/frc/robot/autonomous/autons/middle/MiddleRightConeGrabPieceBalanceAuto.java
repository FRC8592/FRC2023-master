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
import frc.robot.commands.PipelineCommand.Pipeline;

import static frc.robot.autonomous.AutonomousPositions.*;

public class MiddleRightConeGrabPieceBalanceAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);

    private SwerveTrajectory F_TO_Ib = generateTrajectoryFromPoints(
        config
            .setStartVelocity(0.0)
            .setEndVelocity(1.0)
            .setReversed(false),
        GRID_F.getPose(),
        GRID_F.translate(1.0, 0.05),
        GRID_F.translate(3.0, 0.05)
    );

    private SwerveTrajectory Ib_TO_GP3 = generateTrajectoryFromPoints(
        config
            .setStartVelocity(1.0)
            .setEndVelocity(0.0)
            .setReversed(false),
        GRID_F.translate(3.0, 0.05),
        GRID_F.translate(4.5, 0.0)
    );

    private SwerveTrajectory GP3_TO_BM = generateTrajectoryFromPoints(
        config
            .setStartVelocity(0.0)
            .setEndVelocity(0.0)
            .setReversed(true),
        GRID_F.translate(4.5, 0),
        BALANCE_MIDDLE.getPose()
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
                new FollowerCommand(drive, F_TO_Ib.addRotation(Rotation2d.fromDegrees(180), Math.PI, 0.25)),
                new LiftCommand(elevator, Heights.STOWED),
                new IntakeCommand(intake, 3.0),
                new PipelineCommand(vision, Pipeline.CUBE)
            ),
            new JointCommand(
                new FollowerCommand(drive, vision, Ib_TO_GP3.addRotation(Rotation2d.fromDegrees(180))),
                new IntakeCommand(intake)
            ),
            new FollowerCommand(drive, GP3_TO_BM),
            new AutobalanceCommand(drive)
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}