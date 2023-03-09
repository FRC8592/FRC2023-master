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

public class LoadingZoneBalanceAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1.8, 1.0);
    
    private SwerveTrajectory C_TO_BM = generateTrajectoryFromPoints(
        config.setEndVelocity(0.1),
        GRID_C.getPose(),
        INTERMEDIARY_LOADING_ZONE.translate(-2.0, 0.05),
        INTERMEDIARY_LOADING_ZONE.translate(-0.25, -0.1),
        BALANCE_MIDDLE.rotate(Rotation2d.fromDegrees(180.0))
    );

     @Override
    public void initialize() {
        queue = new CommandQueue(
            new FollowerCommand(drive, C_TO_BM), // Drive to charging station
            new AutobalanceCommand(drive) // Balance 
        );
    }
    @Override
    public void periodic() {
        queue.run();
    }

}
