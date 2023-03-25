package frc.robot.autonomous.autons.right;

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

public class RightConeCubeBalanceAuto extends BaseAuto {
    private TrajectoryConfig fastConfig = new TrajectoryConfig(3, 1);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(1, 1);

    private SwerveTrajectory G_TO_Icc = generate(
        fastConfig
            .setStartVelocity(0.0)
            .setEndVelocity(2.0)
            .setReversed(false),
        GRID_G.getPose(),
        INTERMEDIARY_CABLE_COVER.translate(-2.0, -0.2),
        INTERMEDIARY_CABLE_COVER.translate(-1.0, -0.1),
        INTERMEDIARY_CABLE_COVER.translate(0.0, -0.1)
    );

    private SwerveTrajectory Icc_TO_GP4 = generate(
        fastConfig
            .setStartVelocity(2.0)
            .setEndVelocity(0.0)
            .setReversed(false),
        INTERMEDIARY_CABLE_COVER.translate(-0.0, -0.1),
        GAME_PIECE_4.translate(0.25, 0.05)
    );

    private SwerveTrajectory GP4_TO_Icc = generate(
        fastConfig
            .setStartVelocity(0.0)
            .setEndVelocity(1.0)
            .setReversed(true),
        GAME_PIECE_4.translate(0.25, 0.05),
        GAME_PIECE_4.translate(-0.6, 0.0),
        GAME_PIECE_4.translate(-2.0, 0.0),
        GAME_PIECE_4.translate(-3.0, 0.05)
        // GRID_B.translate(0.25 + 2.0, 0)
    );

    private SwerveTrajectory Icc_TO_H = generate(
        slowConfig
            .setStartVelocity(1.0)
            .setEndVelocity(0.0)
            .setReversed(true),
        GAME_PIECE_4.translate(-3.0, 0.05),
        GAME_PIECE_4.translate(-3.9, 0.05)
    );

    private SwerveTrajectory H_TO_BALANCE = generate(
        slowConfig
            .setStartVelocity(0.0)
            .setEndVelocity(1.0)
            .setReversed(false),
        GAME_PIECE_4.translate(-3.9, 0.05),
        BALANCE_MIDDLE.translate(-2.5, 0.0)
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
            new LiftCommand(elevator, Heights.PRIME),
            new JointCommand(
                new FollowerCommand(drive, G_TO_Icc.addRotation(Rotation2d.fromDegrees(180), 2 * Math.PI, 0.5)),
                new LiftCommand(elevator, Heights.STOWED),
                new IntakeCommand(intake, 1.0, true)
            ),
            new JointCommand(
                new FollowerCommand(drive, vision, Icc_TO_GP4.addRotation(Rotation2d.fromDegrees(180), 2 * Math.PI, 0.0).addVision()),
                new IntakeCommand(intake, true)
            ),
            new JointCommand(
                new PipelineCommand(vision, Pipeline.APRIL_TAG),
                new FollowerCommand(drive, GP4_TO_Icc.addRotation(new Rotation2d(), 2 * Math.PI, 1.0)),
                new LiftCommand(elevator, Heights.PRIME)
            ),
            new JointCommand(
                new FollowerCommand(drive, vision, Icc_TO_H.addVision().setAcceptanceRange(0.05)),
                new LiftCommand(elevator, Heights.HIGH),
                new ScoreCommand(intake, 0.5)
            ),
            new LiftCommand(elevator, Heights.PRIME),
            new FollowerCommand(drive, H_TO_BALANCE.addRotation(Rotation2d.fromDegrees(180), 2 * Math.PI, 1.0)),
            new AutobalanceCommand(drive)
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
}