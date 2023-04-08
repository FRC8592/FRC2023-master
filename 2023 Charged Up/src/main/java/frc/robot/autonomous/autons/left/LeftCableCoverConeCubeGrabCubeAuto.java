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

public class LeftCableCoverConeCubeGrabCubeAuto extends BaseAuto {
    private TrajectoryConfig fastConfig = new TrajectoryConfig(4.0, 2.0);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(2.0, 1.0);

    private SwerveTrajectory A_TO_CABLE_COVER = generate(
        slowConfig
            .setStartVelocity(0.0)
            .setEndVelocity(0.75)
            .setReversed(false), 
        GRID_A.getPose(),
        INTERMEDIARY_LOADING_ZONE.translate(-1.5, 0.1)
    ).addRotation(Rotation2d.fromDegrees(180), 2 * Math.PI, 0.5);

    private SwerveTrajectory CABLE_COVER_PASS_1 = generate(
        slowConfig
            .setStartVelocity(0.75)
            .setEndVelocity(1.25)
            .setReversed(false), 
        INTERMEDIARY_LOADING_ZONE.translate(-1.5, 0.1),
        GAME_PIECE_1.translate(-2.0, 0.0)
    ).addRotation(Rotation2d.fromDegrees(180));

    private SwerveTrajectory CABLE_COVER_TO_NEUTRAL_ZONE = generate(
        fastConfig
            .setStartVelocity(1.25)
            .setEndVelocity(2.0)
            .setReversed(false), 
        GAME_PIECE_1.translate(-2.0, 0.0),
        GAME_PIECE_1.translate(-1.0, -0.05)
    ).addRotation(Rotation2d.fromDegrees(180));

    private SwerveTrajectory NEUTRAL_ZONE_TO_GP1 = generate(
        fastConfig
            .setStartVelocity(2.0)
            .setEndVelocity(0.0)
            .setReversed(false), 
        GAME_PIECE_1.translate(-1.0, -0.05),
        GAME_PIECE_1.translate(0.25, -0.05)
    ).addRotation(Rotation2d.fromDegrees(180)).addVision();

    private SwerveTrajectory GP1_TO_CABLE_COVER = generate(
        fastConfig
            .setStartVelocity(0.0)
            .setEndVelocity(0.75)
            .setReversed(true), 
        GAME_PIECE_1.translate(0.25, -0.05),
        INTERMEDIARY_LOADING_ZONE.translate(-0.5, 0.1)
    );

    private SwerveTrajectory CABLE_COVER_PASS_2 = generate(
        slowConfig
            .setStartVelocity(0.75)
            .setEndVelocity(1.25)
            .setReversed(true), 
        INTERMEDIARY_LOADING_ZONE.translate(-0.5, 0.1),
        INTERMEDIARY_LOADING_ZONE.translate(-1.5, 0.1)
    );

    private SwerveTrajectory CABLE_COVER_TO_B = generate(
        slowConfig
            .setStartVelocity(1.25)
            .setEndVelocity(0.0)
            .setReversed(true), 
        INTERMEDIARY_LOADING_ZONE.translate(-1.5, 0.1),
        GRID_B.translate(-0.1, 0.0)
    ).addVision().setAcceptanceRange(0.05);

    private SwerveTrajectory B_TO_CABLE_COVER = generate(
        slowConfig
            .setStartVelocity(0.0)
            .setEndVelocity(0.75)
            .setReversed(false), 
        GRID_B.translate(-0.1, 0.0),
        INTERMEDIARY_LOADING_ZONE.translate(-1.5, 0.1)
    );

    private SwerveTrajectory CABLE_COVER_PASS_3 = generate(
        slowConfig
            .setStartVelocity(0.75)
            .setEndVelocity(3.0)
            .setReversed(false), 
        INTERMEDIARY_LOADING_ZONE.translate(-1.5, 0.1),
        GAME_PIECE_1.translate(-2.0, 0.0)
    );

    private SwerveTrajectory CABLE_COVER_TO_GP2 = generate(
        fastConfig
            .setStartVelocity(0.75)
            .setEndVelocity(0.0)
            .setReversed(false), 
        GAME_PIECE_1.translate(-2.0, 0.0),
        GAME_PIECE_2.translate(-0.05, 0.25, Rotation2d.fromDegrees(-45))
    ).addRotation(Rotation2d.fromDegrees(-45));

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new LiftCommand(elevator, Heights.PRIME), // Prime elevator
            new JointCommand( // Score preload
                new ScoreCommand(intake),
                new LiftCommand(elevator, Heights.HIGH)
            ),
            new JointCommand( // Move to cable cover and stow elevator
                new FollowerCommand(drive, A_TO_CABLE_COVER),
                new LiftCommand(elevator, Heights.STOWED, true),
                new PipelineCommand(vision, Pipeline.CUBE)
            ),
            new FollowerCommand(drive, CABLE_COVER_PASS_1), // Pass over cable cover
            new JointCommand( // Move towards second piece
                new FollowerCommand(drive, CABLE_COVER_TO_NEUTRAL_ZONE),
                new IntakeCommand(intake, true)
            ),
            new JointCommand( // Intake second game piece
                new FollowerCommand(drive, vision, NEUTRAL_ZONE_TO_GP1),
                new IntakeCommand(intake, true)
            ),
            new JointCommand( // Move back to community
                new FollowerCommand(drive, GP1_TO_CABLE_COVER),
                new PipelineCommand(vision, Pipeline.APRIL_TAG),
                new LiftCommand(elevator, Heights.PRIME, true)
            ),
            new FollowerCommand(drive, CABLE_COVER_PASS_2), // Pass over cable cover
            new JointCommand( // Score second game piece
                new FollowerCommand(drive, CABLE_COVER_TO_B),
                new LiftCommand(elevator, Heights.HIGH, true),
                new ScoreCommand(intake, 0.5)
            ),
            new JointCommand( // Move towards cable cover
                new FollowerCommand(drive, B_TO_CABLE_COVER),
                new LiftCommand(elevator, Heights.STOWED, true)
            ),
            new FollowerCommand(drive, CABLE_COVER_PASS_3), // Pass over cable cover
            new JointCommand( // Move towards third game piece and intake
                new FollowerCommand(drive, CABLE_COVER_TO_GP2),
                new IntakeCommand(intake, true)
            )
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
