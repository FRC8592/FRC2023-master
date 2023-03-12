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

public class LoadingZoneTwoPieceAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);

    private SwerveTrajectory C_TO_Ilz = generateTrajectoryFromPoints(
        config
            .setReversed(false)
            .setStartVelocity(0.0)
            .setEndVelocity(1.0),
        GRID_C.getPose(),
        INTERMEDIARY_LOADING_ZONE.translate(-2.0, 0.0),
        INTERMEDIARY_LOADING_ZONE.translate(-0.5, 0.0)
    );

    private SwerveTrajectory Ilz_TO_GP1 = generateTrajectoryFromPoints(
        config
            .setReversed(false)
            .setStartVelocity(1.0)
            .setEndVelocity(0.0),
        INTERMEDIARY_LOADING_ZONE.translate(-0.5, 0.0),
        GAME_PIECE_1.translate(-0.3, -0.1)
    );

    private SwerveTrajectory GP1_TO_Ilz = generateTrajectoryFromPoints(
        config
            .setReversed(true)
            .setStartVelocity(0.0)
            .setEndVelocity(1.0),
        GAME_PIECE_1.translate(-0.3, -0.1),
        INTERMEDIARY_LOADING_ZONE.translate(0.5, 0.0)
        // INTERMEDIARY_LOADING_ZONE.translate(-1.0, 0.1),
        // GRID_A.translate(0.05, -0.3)
        // GRID_A.translate(1.0, 0.0)
    );

    private SwerveTrajectory Ilz_TO_A = generateTrajectoryFromPoints(
        config
            .setReversed(true)
            .setStartVelocity(1.0)
            .setEndVelocity(0.0),
        INTERMEDIARY_LOADING_ZONE.translate(0.5, 0.0),
        INTERMEDIARY_LOADING_ZONE.translate(-1.0, 0.1),
        GRID_A.translate(0.05, -0.3)
    );

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new LiftCommand(elevator, Heights.PRIME), // Tilt up
            new JointCommand( // Lift to high and score pre-load
                new ScoreCommand(intake), 
                new LiftCommand(elevator, Heights.HIGH)
            ),
            new LiftCommand(elevator, Heights.PRIME), // Retract lift
            new JointCommand( // Retract 4-bar and move out community
                new LiftCommand(elevator, Heights.STOWED),
                new FollowerCommand(drive, C_TO_Ilz)
            ),
            new JointCommand( // Turn and move to game piece while intaking
                new FollowerCommand(drive, Ilz_TO_GP1.addRotation(Rotation2d.fromDegrees(180))),
                new IntakeCommand(intake, 0.25, true)
            ),
            new JointCommand( // Continue intaking
                new DelayCommand(0.5),
                new IntakeCommand(intake, true)
            ),
            new FollowerCommand(drive, GP1_TO_Ilz.addRotation(Rotation2d.fromDegrees(180))), // Drive to community line
            new JointCommand( // Turn and drive to grid space A while tilting 4-bar
                new FollowerCommand(drive, Ilz_TO_A),
                new LiftCommand(elevator, Heights.PRIME)
            ), 
            new JointCommand( // Lift to high and score second piece
                new ScoreCommand(intake),
                new LiftCommand(elevator, Heights.HIGH)
            ),
            new LiftCommand(elevator, Heights.STOWED) // Stow elevator
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
