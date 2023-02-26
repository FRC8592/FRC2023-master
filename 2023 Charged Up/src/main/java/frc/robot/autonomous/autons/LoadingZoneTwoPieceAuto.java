package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Elevator.Heights;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JointCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.ScoreCommand;

import static frc.robot.autonomous.AutonomousPositions.*;

public class LoadingZoneTwoPieceAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);

    @Override
    public void initialize() {
        SwerveTrajectory C_TO_Ilz = generateTrajectoryFromPoints(
            config,
            GRID_C.getPose(),
            INTERMEDIARY_LOADING_ZONE.translate(-2.0, 0.0)
        );

        SwerveTrajectory Ilz_TO_GP1 = generateTrajectoryFromPoints(
            config,
            INTERMEDIARY_LOADING_ZONE.translate(-2.0, 0.0),
            INTERMEDIARY_LOADING_ZONE.translate(-1.0, 0.0),
            INTERMEDIARY_LOADING_ZONE.translate(-1.0, 0.0),
            INTERMEDIARY_LOADING_ZONE.translate(0.5, 0.0),
            GAME_PIECE_1.translate(-0.3, -0.2)
        );

        SwerveTrajectory GP1_TO_A = generateTrajectoryFromPoints(
            config.setReversed(true),
            GAME_PIECE_1.translate(-0.3, -0.2),
            INTERMEDIARY_LOADING_ZONE.translate(0.5, 0.0),
            INTERMEDIARY_LOADING_ZONE.translate(-1.0, 0.1),
            GRID_A.translate(0.05, -0.3)
            // GRID_A.translate(1.0, 0.0)
        );

        queue = new CommandQueue(
            new LiftCommand(elevator, Heights.PRIME), // Lifts to high position
            new JointCommand(
                new ScoreCommand(intake), // Pull out the intake and score the pre-loaded piece
                new LiftCommand(elevator, Heights.HIGH)
            ),
            new LiftCommand(elevator, Heights.STOWED), // Retracts the elevator and moves while it stows
            new FollowerCommand(drive, C_TO_Ilz),
            new JointCommand(
                new FollowerCommand(drive, Ilz_TO_GP1.addRotation(Rotation2d.fromDegrees(180))),
                new IntakeCommand(intake, true)
            ),
            // new FollowerCommand(drive, Ilz_TO_GP1.addRotation(Rotation2d.fromDegrees(180))),
            new JointCommand(
                new DelayCommand(1.0),
                new IntakeCommand(intake, true)
            ),
                new FollowerCommand(drive, GP1_TO_A.addRotation(Rotation2d.fromDegrees(0))),
            new LiftCommand(elevator, Heights.PRIME), // Lifts to high position
            new JointCommand(
                new ScoreCommand(intake), // Pull out the intake and score the pre-loaded piece
                new LiftCommand(elevator, Heights.HIGH)
            ),
            new LiftCommand(elevator, Heights.STOWED)
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
