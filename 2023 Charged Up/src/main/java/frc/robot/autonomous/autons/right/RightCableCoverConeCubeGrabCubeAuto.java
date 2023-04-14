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

public class RightCableCoverConeCubeGrabCubeAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(4.0, 1.0);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(1.0, 1.0);

    private SwerveTrajectory G_TO_CABLE_COVER = generate(
        config
            .setStartVelocity(0.0)
            .setEndVelocity(0.75)
            .setReversed(false),
        GRID_G.getPose(),
        INTERMEDIARY_CABLE_COVER.translate(-2.0, -0.2)
    ).addRotation(Rotation2d.fromDegrees(180), 2 * Math.PI, 0.5);

    private SwerveTrajectory CABLE_COVER_TO_Icc = generate(
        config
            .setStartVelocity(0.75)
            .setEndVelocity(2.0)
            .setReversed(false),
        INTERMEDIARY_CABLE_COVER.translate(-2.0, -0.2),
        INTERMEDIARY_CABLE_COVER.translate(0.0, -0.1)
    ).addRotation(Rotation2d.fromDegrees(180));

    private SwerveTrajectory Icc_TO_GP4 = generate(
        config
            .setStartVelocity(2.0)
            .setEndVelocity(0.0)
            .setReversed(false),
        INTERMEDIARY_CABLE_COVER.translate(0.0, -0.1),
        GAME_PIECE_4.translate(0.25, 0.05)
    ).addRotation(Rotation2d.fromDegrees(180)).addVision();

    private SwerveTrajectory GP4_TO_CABLE_COVER = generate(
        config
            .setStartVelocity(0.0)
            .setEndVelocity(1.0)
            .setReversed(true),
        GAME_PIECE_4.translate(0.25, 0.05),
        GAME_PIECE_4.translate(-0.6, 0.0),
        GAME_PIECE_4.translate(-1.75, -0.1)
    ).addRotation(new Rotation2d(), 2 * Math.PI, 0.25);

    private SwerveTrajectory CABLE_COVER_PASS = generate(
        config
            .setStartVelocity(1.0)
            .setEndVelocity(0.5)
            .setReversed(true),
        GAME_PIECE_4.translate(-1.75, -0.1),
        GAME_PIECE_4.translate(-2.5, -0.1)
    );

    private SwerveTrajectory H_TO_SCORE = generate(
        slowConfig
            .setStartVelocity(0.5)
            .setEndVelocity(0.0)
            .setReversed(true),
        GAME_PIECE_4.translate(-2.5, -0.1),
        GAME_PIECE_4.translate(-4.5, 0.05)
    ).addVision().setAcceptanceRange(0.05);

    private SwerveTrajectory H_TO_CABLE_COVER = generate(
        config
        .setStartVelocity(0.0)
        .setEndVelocity(0.75)
        .setReversed(false),
        GAME_PIECE_4.translate(-4.5, 0.05),
        INTERMEDIARY_CABLE_COVER.translate(-2.0, -0.2),
        INTERMEDIARY_CABLE_COVER.translate(0.0, -0.2)
    );

    private SwerveTrajectory CABLE_COVER_TO_GP3 = generate(
        config
            .setStartVelocity(0.75)
            .setEndVelocity(0.0)
            .setReversed(false),
        INTERMEDIARY_CABLE_COVER.translate(0.0, -0.2),
        GAME_PIECE_4.translate(-1.0, 0.05),
        GAME_PIECE_4.translate(-0.5, 0.5),
        GAME_PIECE_3.translate(-0.25, -0.35, Rotation2d.fromDegrees(45))
    ).addRotation(Rotation2d.fromDegrees(45));

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new LiftCommand(elevator, Heights.PRIME), // PRIME 4-bar
            new JointCommand( // Lift elevator HIGH and score
                new ScoreCommand(intake),
                new LiftCommand(elevator, Heights.HIGH)
            ),
            new JointCommand( 
                new FollowerCommand(drive, G_TO_CABLE_COVER),
                new LiftCommand(elevator, Heights.STOWED, true),
                new IntakeCommand(intake, 1.0, true)
            ),
            new JointCommand(
                new FollowerCommand(drive, CABLE_COVER_TO_Icc),
                new IntakeCommand(intake, true),
                new PipelineCommand(vision, Pipeline.CUBE)
            ),
            new JointCommand(
                new FollowerCommand(drive, vision, Icc_TO_GP4),
                new IntakeCommand(intake, true)
            ),
            new JointCommand(
                new FollowerCommand(drive, GP4_TO_CABLE_COVER),
                new LiftCommand(elevator, Heights.PRIME)
            ),
            new JointCommand(
                new PipelineCommand(vision, Pipeline.APRIL_TAG),
                new FollowerCommand(drive, CABLE_COVER_PASS)
            ),
            new JointCommand(
                new FollowerCommand(drive, vision, H_TO_SCORE),
                new LiftCommand(elevator, Heights.HIGH),
                new ScoreCommand(intake, 0.75)
            ),
            new JointCommand(
                new LiftCommand(elevator, Heights.STOWED),
                new FollowerCommand(drive, H_TO_CABLE_COVER)
            ),
            new JointCommand(
                new FollowerCommand(drive, CABLE_COVER_TO_GP3),
                new IntakeCommand(intake)
            )
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
}