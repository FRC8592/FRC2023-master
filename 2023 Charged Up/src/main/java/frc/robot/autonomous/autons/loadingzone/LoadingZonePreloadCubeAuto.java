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

public class LoadingZonePreloadCubeAuto extends BaseAuto {
    private TrajectoryConfig fastConfig = new TrajectoryConfig(3, 2);
    private TrajectoryConfig slowConfig = new TrajectoryConfig(1, 2);

    private SwerveTrajectory A_TO_Ilz = generateTrajectoryFromPoints(
        slowConfig
            .setStartVelocity(0.0)
            .setEndVelocity(1.0)
            .setReversed(false),
        GRID_A.getPose(),
        INTERMEDIARY_LOADING_ZONE.translate(-0.5, 0.4)
    );

    private SwerveTrajectory Ilz_TO_GP1 = generateTrajectoryFromPoints(
        slowConfig
            .setStartVelocity(1.0)
            .setEndVelocity(0.0)
            .setReversed(false),
        INTERMEDIARY_LOADING_ZONE.translate(-0.5, 0.4),
        GAME_PIECE_1.translate(-0.5, 0.0)
    );

    private SwerveTrajectory GP1_TO_B = generateTrajectoryFromPoints(
        fastConfig
            .setStartVelocity(0.0)
            .setEndVelocity(0.0)
            .setReversed(true),
        GAME_PIECE_1.translate(-0.5, 0.0),
        GAME_PIECE_1.translate(-4.0, 0.0)
        // GRID_B.translate(0.25, 0.0)
    );

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new LiftCommand(elevator, Heights.PRIME), // Tilt up 4-bar
            new JointCommand( // Lift to HIGH and score pre-load
                new LiftCommand(elevator, Heights.HIGH),
                new ScoreCommand(intake)
            ),
            new LiftCommand(elevator, Heights.PRIME), // Retract to STOWED
            new JointCommand( // Tilt down 4-bar and drive to edge of community
                new FollowerCommand(drive, A_TO_Ilz),
                new LiftCommand(elevator, Heights.STOWED)
            ),
            new JointCommand( // Drive to cube while turning and intaking and continue stowing if not stowed yet
                new FollowerCommand(drive, Ilz_TO_GP1.addRotation(Rotation2d.fromDegrees(180))),
                new IntakeCommand(intake, true),
                new LiftCommand(elevator, Heights.STOWED)
            ),
            new DelayCommand(0.5), // Wait for wrist to mostly raise
            new JointCommand( // Drive back to grid space B and tilt up 4-bar
                new FollowerCommand(drive, GP1_TO_B),
                new LiftCommand(elevator, Heights.PRIME)    
            ),
            new JointCommand( // Lift to HIGH and score cube
                new LiftCommand(elevator, Heights.HIGH),
                new ScoreCommand(intake)
            ),
            new LiftCommand(elevator, Heights.PRIME) // Retract to STOWED
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
