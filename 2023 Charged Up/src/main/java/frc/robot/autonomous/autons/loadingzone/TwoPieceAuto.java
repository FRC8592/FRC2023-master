package frc.robot.autonomous.autons.loadingzone;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Elevator.Heights;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.autonomous.autons.BaseAuto;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JointCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.PipelineCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.PipelineCommand.Pipeline;

import static frc.robot.autonomous.AutonomousPositions.*;

public class TwoPieceAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(3.0, 1);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(1.0, 1);

    private SwerveTrajectory C_TO_Ilz = generateTrajectoryFromPoints(
        config
            .setStartVelocity(0.0)
            .setEndVelocity(2.0)
            .setReversed(false),
        GRID_C.getPose(),
        INTERMEDIARY_LOADING_ZONE.translate(-2.0, 0.2),
        INTERMEDIARY_LOADING_ZONE.translate(-1.0, 0.1),
        INTERMEDIARY_LOADING_ZONE.translate(0.0, 0.1)
    );

    private SwerveTrajectory Ilz_TO_GP1 = generateTrajectoryFromPoints(
        config
            .setStartVelocity(2.0)
            .setEndVelocity(0.0)
            .setReversed(false),
        INTERMEDIARY_LOADING_ZONE.translate(-0.0, 0.1),
        GAME_PIECE_1.translate(-0.1, -0.05)
    );

    private SwerveTrajectory GP1_TO_B = generateTrajectoryFromPoints(
        config
            .setStartVelocity(0.0)
            .setEndVelocity(1.0)
            .setReversed(true),
        GAME_PIECE_1.translate(-0.1, -0.05),
        GAME_PIECE_1.translate(-0.6, 0.0),
        GAME_PIECE_1.translate(-2.0, 0.0),
        GAME_PIECE_1.translate(-3.3, -0.05)
        // GRID_B.translate(0.25 + 2.0, 0)
    );

    private SwerveTrajectory B_TO_SCORE = generateTrajectoryFromPoints(
        slowConfig
            .setStartVelocity(1.0)
            .setEndVelocity(0.0)
            .setReversed(true),
        // GRID_B.translate(0.25 + 2.0, 0),
        GAME_PIECE_1.translate(-3.3, -0.05),
        GAME_PIECE_1.translate(-4.0, -0.05)
    );

    @Override
    public void initialize() {
        queue = new CommandQueue(
            // new LiftCommand(elevator, Heights.PRIME),
            // new JointCommand(
            //     new LiftCommand(elevator, Heights.STOWED),
            //     new FollowerCommand(drive, C_TO_Ilz.addRotation(Rotation2d.fromDegrees(180), 2 * Math.PI, 0.5))
            // ),
            // new JointCommand(
            //     new IntakeCommand(intake),
            //     new FollowerCommand(drive, Ilz_TO_GP1.addRotation(Rotation2d.fromDegrees(180), 2 * Math.PI, 0.5))
            // ),
            // new JointCommand(
            //     new FollowerCommand(drive, GP1_TO_B.addRotation(Rotation2d.fromDegrees(0), 2 * Math.PI, 0.5)),
            //     new LiftCommand(elevator, Heights.PRIME)  
            // ),
            // new JointCommand(
            //     new ScoreCommand(intake),
            //     new LiftCommand(elevator, Heights.HIGH)
            // ),
            // new LiftCommand(elevator, Heights.STOWED)
            new LiftCommand(elevator, Heights.PRIME),
            new JointCommand(
                new ScoreCommand(intake),
                new LiftCommand(elevator, Heights.HIGH)
            ),
            new LiftCommand(elevator, Heights.PRIME),
            new PipelineCommand(vision, Pipeline.CUBE),
            new JointCommand(
                new LiftCommand(elevator, Heights.STOWED),
                new FollowerCommand(drive, C_TO_Ilz.addRotation(Rotation2d.fromDegrees(180), 2 * Math.PI, 0.5)),
                new IntakeCommand(intake, 1.0, true)
            ),
            new JointCommand(
                new FollowerCommand(drive, vision, Ilz_TO_GP1.addRotation(Rotation2d.fromDegrees(180), 2 * Math.PI, 0.0).addVision()),
                new IntakeCommand(intake, true)
            ),
            new JointCommand(
                new PipelineCommand(vision, Pipeline.APRIL_TAG),
                new FollowerCommand(drive, GP1_TO_B)
            ),
            new JointCommand(
                new FollowerCommand(drive, vision, B_TO_SCORE.addVision().setAcceptanceRange(0.05)),
                new LiftCommand(elevator, Heights.PRIME)
            ),
            new JointCommand(
                new LiftCommand(elevator, Heights.HIGH),
                new ScoreCommand(intake)
            ),
            new LiftCommand(elevator, Heights.STOWED)
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}