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

public class LoadingZonePreloadConeAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);

    private SwerveTrajectory C_TO_GP1 = generateTrajectoryFromPoints(
        config.setReversed(false),
        GRID_C.getPose(),
        INTERMEDIARY_LOADING_ZONE.translate(-2.0, 0.0),
        INTERMEDIARY_LOADING_ZONE.translate(0.5, 0.0),
        GAME_PIECE_1.translate(-0.5, 0)
    );

    private SwerveTrajectory GP1_TO_B = generateTrajectoryFromPoints(
        config.setReversed(true),
        GAME_PIECE_1.translate(-0.5, 0),
        GRID_B.translate(0.25, 0)
    );

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new LiftCommand(elevator, Heights.PRIME), // Tilt up
            new JointCommand( // Lift to HIGH and score
                new LiftCommand(elevator, Heights.HIGH),
                new ScoreCommand(intake)
            ),
            new LiftCommand(elevator, Heights.PRIME), // Lift to PRIME
            new JointCommand( // Stow elevator, move to game piece and start intaking
                new LiftCommand(elevator, Heights.STOWED),
                new FollowerCommand(drive, C_TO_GP1.addRotation(Rotation2d.fromDegrees(180), Math.PI, 1.0)),
                new IntakeCommand(intake, 2.0)
            ),
            new JointCommand( // Move back to scoring grid and prime elevator
                new FollowerCommand(drive, GP1_TO_B.addRotation(Rotation2d.fromDegrees(0), Math.PI, 1.0)),
                new LiftCommand(elevator, Heights.PRIME, 1.0)
            ),
            new JointCommand( // Lift to HIGH and score
                new LiftCommand(elevator, Heights.HIGH),
                new ScoreCommand(intake)
            ),
            new LiftCommand(elevator, Heights.STOWED) // Stow elevator
        );
    }

    @Override
    public void periodic() {
        queue.run();        
    }
    
}
