package frc.robot.autonomous.autons.right;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Elevator.Heights;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.autonomous.autons.BaseAuto;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JointCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.PipelineCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.PipelineCommand.Pipeline;
import static frc.robot.autonomous.AutonomousPositions.*;

public class RightCableCoverConeCubeAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(3.0, 1.0);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(1.0, 1.0);

    private SwerveTrajectory G_TO_CABLE_COVER = generate(
        config
            .setStartVelocity(0.0)
            .setEndVelocity(1.0)
            .setReversed(false),
        GRID_G.getPose(),
        INTERMEDIARY_CABLE_COVER.translate(-2.0, -0.2),
        INTERMEDIARY_CABLE_COVER.translate(-1.0, -0.1)
    ).addRotation(Rotation2d.fromDegrees(180), 2 * Math.PI, 0.5);

    private SwerveTrajectory CABLE_COVER_TO_Icc = generate(
        config
            .setStartVelocity(1.0)
            .setEndVelocity(2.0)
            .setReversed(false),
        INTERMEDIARY_CABLE_COVER.translate(-1.0, -0.1),
        INTERMEDIARY_CABLE_COVER.translate(0.0, -0.1)
    ).addRotation(Rotation2d.fromDegrees(180));

    private SwerveTrajectory Icc_TO_GP4 = generate(
        config
            .setStartVelocity(2.0)
            .setEndVelocity(0.0)
            .setReversed(false),
        INTERMEDIARY_CABLE_COVER.translate(-0.0, -0.1),
        GAME_PIECE_4.translate(0.25, 0.05)
    ).addRotation(Rotation2d.fromDegrees(180)).addVision();

    private SwerveTrajectory GP4_TO_H = generate(
        config
            .setStartVelocity(0.0)
            .setEndVelocity(1.0)
            .setReversed(true),
        GAME_PIECE_4.translate(0.25, 0.05),
        GAME_PIECE_4.translate(-0.6, 0.0),
        GAME_PIECE_4.translate(-2.0, 0.0),
        GAME_PIECE_4.translate(-3.0, 0.05)
    );

    private SwerveTrajectory H_TO_SCORE = generate(
        slowConfig
            .setStartVelocity(1.0)
            .setEndVelocity(0.0)
            .setReversed(true),
        GAME_PIECE_4.translate(-3.0, 0.05),
        GAME_PIECE_4.translate(-4.2, 0.05)
    ).addVision().setAcceptanceRange(0.05);

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new LiftCommand(elevator, Heights.PRIME), // PRIME 4-bar
            new JointCommand( // Lift elevator HIGH and score
                new ScoreCommand(intake),
                new LiftCommand(elevator, Heights.HIGH)
            ),
            new LiftCommand(elevator, Heights.PRIME), // Lift elevator PRIME
            new PipelineCommand(vision, Pipeline.CUBE), // Change pipeline CUBE
            new JointCommand(
                new LiftCommand(elevator, Heights.STOWED),
                new FollowerCommand(drive, G_TO_CABLE_COVER)
            ),
            new JointCommand( // STOW 4-bar and INTAKE while moving out community
                new FollowerCommand(drive, CABLE_COVER_TO_Icc),
                new IntakeCommand(intake, true)
            ),
            new JointCommand( // TRACK and INTAKE cube
                new FollowerCommand(drive, vision, Icc_TO_GP4),
                new IntakeCommand(intake, true)
            ),
            new JointCommand( // Change pipeline APRIL TAG and PRIME 4-bar while moving back to community
                new PipelineCommand(vision, Pipeline.APRIL_TAG),
                new FollowerCommand(drive, GP4_TO_H),
                new LiftCommand(elevator, Heights.PRIME)
            ),
            new JointCommand( // TRACK grid and continue PRIME 4-bar
                new FollowerCommand(drive, vision, H_TO_SCORE),
                new LiftCommand(elevator, Heights.PRIME)
            ),
            new JointCommand( // Lift elevator HIGH and score
                new LiftCommand(elevator, Heights.HIGH),
                new ScoreCommand(intake)
            ),
            new LiftCommand(elevator, Heights.STOWED) // STOW all
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
}