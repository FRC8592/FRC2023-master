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

public class LeftConeCubeBalanceAuto extends BaseAuto {
    private TrajectoryConfig fastConfig = new TrajectoryConfig(3.75, 1);
    private TrajectoryConfig balanceConfig = new TrajectoryConfig(1.0, 1);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(2, 1);

    private SwerveTrajectory C_TO_Ilz = generate(
        fastConfig
            .setStartVelocity(0.0)
            .setEndVelocity(3.0)
            .setReversed(false),
        GRID_C.getPose(),
        INTERMEDIARY_LOADING_ZONE.translate(-2.5, 0.1),
        INTERMEDIARY_LOADING_ZONE.translate(-2.0, 0.1),
        INTERMEDIARY_LOADING_ZONE.translate(-1.0, 0.1),
        INTERMEDIARY_LOADING_ZONE.translate(0.0, 0.1)
    );

    private SwerveTrajectory Ilz_TO_GP1 = generate(
        fastConfig
            .setStartVelocity(3.0)
            .setEndVelocity(0.0)
            .setReversed(false),
        INTERMEDIARY_LOADING_ZONE.translate(0.0, 0.1),
        GAME_PIECE_1.translate(0.25, -0.05)
    );

    private SwerveTrajectory GP1_TO_Ilz = generate(
        fastConfig
            .setStartVelocity(0.0)
            .setEndVelocity(1.5)
            .setReversed(true),
        GAME_PIECE_1.translate(0.25, -0.05),
        GAME_PIECE_1.translate(-0.6, 0.0),
        GAME_PIECE_1.translate(-2.0, 0.0),
        GAME_PIECE_1.translate(-2.5, -0.05)
        // GRID_B.translate(0.25 + 2.0, 0)
    );

    private SwerveTrajectory Ilz_TO_B = generate(
        slowConfig
            .setStartVelocity(1.5)
            .setEndVelocity(0.0)
            .setReversed(true),
        GAME_PIECE_1.translate(-2.5, -0.05),
        GAME_PIECE_1.translate(-3.9, -0.05)
    );

    private SwerveTrajectory B_TO_BALANCE = generate(
        balanceConfig
            .setStartVelocity(0.0)
            .setEndVelocity(1.0)
            .setReversed(false),
        GAME_PIECE_1.translate(-3.9, -0.05),
        GAME_PIECE_1.translate(-3.9, -0.1),
        GAME_PIECE_1.translate(-3.7, -1.0),
        GAME_PIECE_1.translate(-3.5, -1.4),
        GAME_PIECE_1.translate(-2.4, -1.4)
    );

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new LiftCommand(elevator, Heights.PRIME),
            new JointCommand(
                new LiftCommand(elevator, Heights.HIGH),
                new ScoreCommand(intake),
                new PipelineCommand(vision, Pipeline.CUBE)
            ),
            new JointCommand(
                new FollowerCommand(drive, C_TO_Ilz.addRotation(Rotation2d.fromDegrees(180), 2 * Math.PI, 0.5)),
                new LiftCommand(elevator, Heights.STOWED),
                new IntakeCommand(intake, 1.0, true)
            ),
            new JointCommand(
                new LiftCommand(elevator, Heights.STOWED),
                new FollowerCommand(drive, vision, Ilz_TO_GP1.addRotation(Rotation2d.fromDegrees(180), 2 * Math.PI, 0.0).addVision()),
                new IntakeCommand(intake, true)
            ),
            new JointCommand(
                new PipelineCommand(vision, Pipeline.APRIL_TAG),
                new FollowerCommand(drive, GP1_TO_Ilz.addRotation(new Rotation2d(), 2 * Math.PI, 0.25)),
                new LiftCommand(elevator, Heights.PRIME)
            ),
            new JointCommand(
                new FollowerCommand(drive, vision, Ilz_TO_B.addVision().setAcceptanceRange(0.05)),
                new LiftCommand(elevator, Heights.HIGH),
                new ScoreCommand(intake, 0.5)
            ),
            new JointCommand(
                new LiftCommand(elevator, Heights.STOWED),
                new FollowerCommand(drive, B_TO_BALANCE.addRotation(Rotation2d.fromDegrees(180), 2 * Math.PI, 0.75))
            ),
            new JointCommand(
                new AutobalanceCommand(drive),
                new LiftCommand(elevator, Heights.STOWED)
            )
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
