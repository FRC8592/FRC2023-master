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
import frc.robot.commands.ScoreCommand;

import static frc.robot.autonomous.AutonomousPositions.*;

public class TwoPieceAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(2.0, 1);

    private SwerveTrajectory C_TO_Ilz = generateTrajectoryFromPoints(
        config
            .setStartVelocity(0.0)
            .setEndVelocity(1.0)
            .setReversed(false),
        GRID_C.getPose(),
        INTERMEDIARY_LOADING_ZONE.translate(-2.0, 0.2)
    );

    private SwerveTrajectory Ilz_TO_GP1 = generateTrajectoryFromPoints(
        config
            .setStartVelocity(1.0)
            .setEndVelocity(0.0)
            .setReversed(false),
        INTERMEDIARY_LOADING_ZONE.translate(-2.0, 0.2),
        GAME_PIECE_1.translate(-0.1, -0.05)
    );

    private SwerveTrajectory GP1_TO_B = generateTrajectoryFromPoints(
        config
            .setStartVelocity(0.0)
            .setEndVelocity(0.0)
            .setReversed(true),
        GAME_PIECE_1.translate(-0.1, -0.05),
        GRID_B.translate(0.25, 0)
    );

    @Override
    public void initialize() {
        queue = new CommandQueue(
            // new LiftCommand(elevator, Heights.PRIME),
            // new JointCommand(
            //     new ScoreCommand(intake),
            //     new LiftCommand(elevator, Heights.HIGH)
            // ),
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
            new FollowerCommand(drive, C_TO_Ilz.addLogger(logger)),//.addRotation(Rotation2d.fromDegrees(180), 2 * Math.PI, 0.5)),
            new FollowerCommand(drive, Ilz_TO_GP1)//.addRotation(Rotation2d.fromDegrees(180), 2 * Math.PI, 0.5))
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
