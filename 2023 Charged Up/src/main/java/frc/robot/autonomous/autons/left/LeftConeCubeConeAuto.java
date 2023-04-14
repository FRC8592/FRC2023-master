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

public class LeftConeCubeConeAuto extends BaseAuto {
    private TrajectoryConfig fastConfig = new TrajectoryConfig(1.0, 2.0);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(2.0, 1.0);

    private SwerveTrajectory C_TO_NEUTRAL_ZONE = generate(
        fastConfig
            .setStartVelocity(0.0)
            .setEndVelocity(fastConfig.getMaxVelocity())
            .setReversed(false), 
        GRID_C.getPose(),
        GRID_B.translate(0.5, 0.0),
        INTERMEDIARY_LOADING_ZONE.translate(-1.5, 0.1),
        INTERMEDIARY_LOADING_ZONE.translate(-1.0, 0.1),
        INTERMEDIARY_LOADING_ZONE.translate(0.5, 0.1)
    ).addRotation(Rotation2d.fromDegrees(180), 0.5);

    private SwerveTrajectory NEUTRAL_ZONE_TO_GP1 = generate(
        fastConfig
            .setStartVelocity(fastConfig.getMaxVelocity())
            .setEndVelocity(0.0)
            .setReversed(false), 
        INTERMEDIARY_LOADING_ZONE.translate(0.5, 0.1),
        GAME_PIECE_1.translate(0.25, 0.1)
    ).addRotation(Rotation2d.fromDegrees(180)).addVision();

    private SwerveTrajectory GP1_TO_COMMUNITY = generate(
        fastConfig
            .setStartVelocity(0.0)
            .setEndVelocity(2.0)
            .setReversed(true), 
        GAME_PIECE_1.translate(0.25, 0.1),
        INTERMEDIARY_LOADING_ZONE.translate(0.5, 0.0),
        INTERMEDIARY_LOADING_ZONE.translate(-1.0, 0.0)
    );

    private SwerveTrajectory COMMUNITY_TO_B = generate(
        slowConfig
            .setStartVelocity(2.0)
            .setEndVelocity(0.0)
            .setReversed(true), 
        INTERMEDIARY_LOADING_ZONE.translate(-1.0, 0.0),
        GRID_B.translate(0.2, 0.0)
    ).addVision().setAcceptanceRange(0.1);

    private SwerveTrajectory B_TO_NEUTRAL_ZONE = generate(
        fastConfig
            .setStartVelocity(0.0)
            .setEndVelocity(2.0)
            .setReversed(false), 
        GRID_B.translate(0.2, 0.0),
        INTERMEDIARY_LOADING_ZONE.translate(-1.5, 0.0),
        INTERMEDIARY_LOADING_ZONE.translate(0.5, 0.0),
        GAME_PIECE_1.translate(-0.5, -0.5, Rotation2d.fromDegrees(135)),
        GAME_PIECE_2.translate(-0.25, 0.25, Rotation2d.fromDegrees(135))
    ).addRotation(Rotation2d.fromDegrees(135), Math.PI, 1.0);

    private SwerveTrajectory NEUTRAL_ZONE_TO_GP2 = generate(
        fastConfig
            .setStartVelocity(2.0)
            .setEndVelocity(0.0)
            .setReversed(false), 
        GAME_PIECE_2.translate(-0.25, 0.25, Rotation2d.fromDegrees(-45)),
        GAME_PIECE_2.translate(0.25, -0.25, Rotation2d.fromDegrees(-45))
    ).addRotation(Rotation2d.fromDegrees(135), Math.PI / 2, 0.0).addVision();

    private SwerveTrajectory GP2_TO_COMMUNITY = generate(
        fastConfig
            .setStartVelocity(0.0)
            .setEndVelocity(2.0)
            .setReversed(true), 
        GAME_PIECE_2.translate(0.25, -0.1, Rotation2d.fromDegrees(-45)),
        INTERMEDIARY_LOADING_ZONE.translate(0.5, 0.1),
        INTERMEDIARY_LOADING_ZONE.translate(-1.5, 0.2)
    ).addRotation(new Rotation2d(), Math.PI / 2, 0.0);

    private SwerveTrajectory COMMUNITY_TO_A = generate(
        slowConfig
            .setStartVelocity(2.0)
            .setEndVelocity(0.0)
            .setReversed(true), 
        INTERMEDIARY_LOADING_ZONE.translate(-1.5, 0.2),
        INTERMEDIARY_LOADING_ZONE.translate(-2.0, 0.2),
        GRID_A.translate(0.6, -0.2)
    ).addRotation(new Rotation2d(), Math.PI / 2, 0.0);

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new FollowerCommand(drive, C_TO_NEUTRAL_ZONE)
            // new JointCommand( // Score preload
            //     new LiftCommand(elevator, Heights.MID),
            //     new ScoreCommand(intake, 0.5)
            // ),
            // new JointCommand( // Move to second piece
            //     new FollowerCommand(drive, C_TO_NEUTRAL_ZONE),
            //     new LiftCommand(elevator, Heights.STOWED),
            //     new IntakeCommand(intake, 1.5, true),
            //     new PipelineCommand(vision, Pipeline.CUBE)
            // ),
            // new JointCommand( // Intake cube
            //     new FollowerCommand(drive, vision, NEUTRAL_ZONE_TO_GP1),
            //     new IntakeCommand(intake, true)
            // )
            // new JointCommand( // Move back to community
            //     new FollowerCommand(drive, GP1_TO_COMMUNITY),
            //     new LiftCommand(elevator, Heights.PRIME, 1.0, true),
            //     new PipelineCommand(vision, Pipeline.APRIL_TAG)
            // ),
            // new JointCommand( // Score second piece
            //     new FollowerCommand(drive, vision, COMMUNITY_TO_B),
            //     new LiftCommand(elevator, Heights.MID),
            //     new ScoreCommand(intake, 0.25)
            // ),
            // new JointCommand( // Move to third piece
            //     new FollowerCommand(drive, B_TO_NEUTRAL_ZONE),
            //     new LiftCommand(elevator, Heights.STOWED, true),
            //     new IntakeCommand(intake, 1.5, true),
            //     new PipelineCommand(vision, Pipeline.CONE)
            // ),
            // new JointCommand( // Intake third piece
            //     new FollowerCommand(drive, NEUTRAL_ZONE_TO_GP2),
            //     new IntakeCommand(intake, true)
            // ),
            // new JointCommand( // Move back to community
            //     new FollowerCommand(drive, GP2_TO_COMMUNITY),
            //     new LiftCommand(elevator, Heights.PRIME, 1.0, true),
            //     new PipelineCommand(vision, Pipeline.RETRO_TAPE)
            // ),
            // new JointCommand( // Score third piece
            //     new FollowerCommand(drive, COMMUNITY_TO_A),
            //     new LiftCommand(elevator, Heights.MID, true),
            //     new ScoreCommand(intake, 0.5)
            // ),
            // new LiftCommand(elevator, Heights.STOWED) // Stow elevator
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
