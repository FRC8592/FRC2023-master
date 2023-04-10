package frc.robot.autonomous.autons.left;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Elevator.Heights;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.autonomous.autons.BaseAuto;
import frc.robot.commands.AutobalanceCommand;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.JointCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.ScoreCommand;
import static frc.robot.autonomous.AutonomousPositions.*;

public class LeftConeCubeCubeAuto extends BaseAuto {
    private TrajectoryConfig fastConfig = new TrajectoryConfig(4.0, 2.0);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(2.0, 2.0);

    private SwerveTrajectory A_TO_NEUTRAL_ZONE = generate(
        fastConfig
            .setStartVelocity(0.0)
            .setEndVelocity(4.0)
            .setReversed(false),
        GRID_A.getPose(),
        INTERMEDIARY_LOADING_ZONE.translate(-1.5, 0.1),
        INTERMEDIARY_LOADING_ZONE.translate(0.5, 0.1)
    ).addRotation(Rotation2d.fromDegrees(180), Math.PI / 2, 0.5);

    private SwerveTrajectory NEUTRAL_ZONE_TO_GP1 = generate(
        fastConfig
            .setStartVelocity(4.0)
            .setEndVelocity(0.0)
            .setReversed(false),
        INTERMEDIARY_LOADING_ZONE.translate(0.5, 0.1),
        GAME_PIECE_1.translate(0.25, 0.0)
    ).addRotation(Rotation2d.fromDegrees(180), Math.PI / 2, 0.0);

    private SwerveTrajectory GP1_TO_COMMUNITY = generate(
        fastConfig
            .setStartVelocity(0.0)
            .setEndVelocity(2.0)
            .setReversed(true),
        GAME_PIECE_1.translate(0.25, 0.0),
        INTERMEDIARY_LOADING_ZONE.translate(0.5, 0.1),
        INTERMEDIARY_LOADING_ZONE.translate(-1.5, 0.1)
    ).addRotation(Rotation2d.fromDegrees(0), Math.PI/2, 0.0);

    private SwerveTrajectory COMMUNITY_TO_B = generate(
        slowConfig
            .setStartVelocity(2.0)
            .setEndVelocity(0.0)
            .setReversed(true),
        INTERMEDIARY_LOADING_ZONE.translate(-1.5, 0.1),
        GRID_B.translate(0.2, 0.1)
    ).addRotation(Rotation2d.fromDegrees(0), Math.PI/2, 0.0);

    private SwerveTrajectory B_TO_NEUTRAL_ZONE = generate(
        slowConfig
            .setStartVelocity(0.0)
            .setEndVelocity(2.0)
            .setReversed(false),
        GRID_B.translate(0.2, 0.1),
        INTERMEDIARY_LOADING_ZONE.translate(-1.5, 0.1),
        INTERMEDIARY_LOADING_ZONE.translate(-1.0, 0.1)
    );

    private SwerveTrajectory NEUTRAL_ZONE_TO_GP2 = generate(
        fastConfig
            .setStartVelocity(2.0)
            .setEndVelocity(1.0)
            .setReversed(false),
        INTERMEDIARY_LOADING_ZONE.translate(-1.0, 0.1),
        GAME_PIECE_2.translate(-0.25, 0.0, Rotation2d.fromDegrees(-45))
    ).addRotation(Rotation2d.fromDegrees(-45));

    private SwerveTrajectory GP2_TO_BM = generate(
        fastConfig
            .setStartVelocity(1.0)
            .setEndVelocity(1.0)
            .setReversed(true),
        GAME_PIECE_2.translate(-0.25, 0.0),
        BALANCE_MIDDLE.translate(-1.2, 0.0)
    );

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new DelayCommand(3.0),
            new FollowerCommand(drive, A_TO_NEUTRAL_ZONE),
            new FollowerCommand(drive, NEUTRAL_ZONE_TO_GP1),
            new FollowerCommand(drive, GP1_TO_COMMUNITY),
            new FollowerCommand(drive, COMMUNITY_TO_B),
            new FollowerCommand(drive, B_TO_NEUTRAL_ZONE),
            new FollowerCommand(drive, NEUTRAL_ZONE_TO_GP2),
            new FollowerCommand(drive, GP2_TO_BM)
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
