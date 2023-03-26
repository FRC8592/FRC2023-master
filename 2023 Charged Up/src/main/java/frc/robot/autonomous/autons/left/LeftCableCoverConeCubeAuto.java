package frc.robot.autonomous.autons.left;

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

public class LeftCableCoverConeCubeAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(3.75, 1.0);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(1.0, 1.0);

    private SwerveTrajectory C_TO_CABLE_COVER = generate(
        config
            .setStartVelocity(0.0)
            .setEndVelocity(0.75)
            .setReversed(false),
        GRID_C.getPose(),
        INTERMEDIARY_LOADING_ZONE.translate(-2.0, 0.2)
    ).addRotation(Rotation2d.fromDegrees(180), 2 * Math.PI, 0.5);

    private SwerveTrajectory CABLE_COVER_TO_Ilz = generate(
        config
            .setStartVelocity(0.75)
            .setEndVelocity(2.0)
            .setReversed(false),
        INTERMEDIARY_LOADING_ZONE.translate(-2.0, 0.2),
        INTERMEDIARY_LOADING_ZONE.translate(0.0, 0.1)
    ).addRotation(Rotation2d.fromDegrees(180));

    private SwerveTrajectory Ilz_TO_GP1 = generate(
        config
            .setStartVelocity(2.0)
            .setEndVelocity(0.0)
            .setReversed(false),
        INTERMEDIARY_LOADING_ZONE.translate(-0.0, 0.1),
        GAME_PIECE_1.translate(0.25, -0.05)
    ).addRotation(Rotation2d.fromDegrees(180)).addVision();

    private SwerveTrajectory GP1_TO_B = generate(
        config
            .setStartVelocity(0.0)
            .setEndVelocity(1.0)
            .setReversed(true),
        GAME_PIECE_1.translate(0.25, -0.05),
        GAME_PIECE_1.translate(-0.6, 0.0),
        GAME_PIECE_1.translate(-1.75, 0.0)
    );

    private SwerveTrajectory CABLE_COVER_PASS = generate(
        config
            .setStartVelocity(1.0)
            .setEndVelocity(0.5)
            .setReversed(true),
        GAME_PIECE_1.translate(-1.75, 0.0),
        GAME_PIECE_1.translate(-2.5, 0.0)
    );

    private SwerveTrajectory B_TO_SCORE = generate(
        slowConfig
            .setStartVelocity(0.5)
            .setEndVelocity(0.0)
            .setReversed(true),
        GAME_PIECE_1.translate(-2.5, 0.0),
        GAME_PIECE_1.translate(-4.0, -0.05)
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
                new FollowerCommand(drive, C_TO_CABLE_COVER)
            ),
            new JointCommand( // STOW 4-bar and INTAKE while moving out community
                new FollowerCommand(drive, CABLE_COVER_TO_Ilz),
                new IntakeCommand(intake, true)
            ),
            new JointCommand( // TRACK and INTAKE cube
                new FollowerCommand(drive, vision, Ilz_TO_GP1),
                new IntakeCommand(intake, true)
            ),
            new JointCommand( // Change pipeline APRIL TAG and PRIME 4-bar while moving back to community
                new PipelineCommand(vision, Pipeline.APRIL_TAG),
                new FollowerCommand(drive, GP1_TO_B),
                new LiftCommand(elevator, Heights.PRIME)
            ),
            new JointCommand(
                new FollowerCommand(drive, vision, CABLE_COVER_PASS),
                new LiftCommand(elevator, Heights.PRIME)
            ),
            new JointCommand( // TRACK grid and continue PRIME 4-bar
                new FollowerCommand(drive, vision, B_TO_SCORE),
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