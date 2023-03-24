package frc.robot.autonomous.autons.loadingzone;

import frc.robot.Elevator.Heights;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.autonomous.autons.BaseAuto;
import frc.robot.commands.AutobalanceCommand;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.JointCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.ScoreCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import static frc.robot.autonomous.AutonomousPositions.*;

public class LeftSidePreloadBalanceAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1.8, 1.0);
    
    private SwerveTrajectory C_TO_Ilz = generate(
        config
            .setStartVelocity(0.0)
            .setEndVelocity(1.0),
        GRID_C.getPose(),
        INTERMEDIARY_LOADING_ZONE.translate(-2.25, 0.0),
        INTERMEDIARY_LOADING_ZONE.translate(-1.0, 0.0),
        INTERMEDIARY_LOADING_ZONE.getPose()
    );

    private SwerveTrajectory Ilz_TO_BM = generate(
        config
            .setStartVelocity(1.0)
            .setEndVelocity(0.1),
        INTERMEDIARY_LOADING_ZONE.getPose(),
        BALANCE_MIDDLE.rotate(Rotation2d.fromDegrees(180))
    );

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new LiftCommand(elevator, Heights.PRIME), // PRIME 4-bar
            new JointCommand( // Lift elevator HIGH and score
                new ScoreCommand(intake), 
                new LiftCommand(elevator, Heights.HIGH)
            ),
            new LiftCommand(elevator, Heights.PRIME), // PRIME elevator
            new JointCommand( // STOW elevator while moving out community
                new LiftCommand(elevator, Heights.STOWED),
                new FollowerCommand(drive, C_TO_Ilz)
            ),
            new FollowerCommand(drive, Ilz_TO_BM), // Move towards charging station
            new AutobalanceCommand(drive) // Balance
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
}