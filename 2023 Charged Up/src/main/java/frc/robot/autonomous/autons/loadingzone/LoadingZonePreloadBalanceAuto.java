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

public class LoadingZonePreloadBalanceAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1.8, 1.0);
    
    private SwerveTrajectory C_TO_BM = generateTrajectoryFromPoints(
        config.setEndVelocity(0.1),
        GRID_C.getPose(),
        INTERMEDIARY_LOADING_ZONE.translate(-2.0, 0.0),
        INTERMEDIARY_LOADING_ZONE.translate(-1.0, 0.0),
        INTERMEDIARY_LOADING_ZONE.translate(0.0, 0.0),
        BALANCE_MIDDLE.translate(0.0, 0.0, Rotation2d.fromDegrees(180))
    );

     @Override
    public void initialize() {
        queue = new CommandQueue(
            new LiftCommand(elevator, Heights.PRIME), // Tilt up
            new JointCommand(
                new ScoreCommand(intake), // Pull out the intake and score the pre-loaded piece
                new LiftCommand(elevator, Heights.HIGH)
            ),
            new LiftCommand(elevator, Heights.PRIME),
            new JointCommand(
                new LiftCommand(elevator, Heights.STOWED),
                new FollowerCommand(drive, C_TO_BM)
            ),
            new AutobalanceCommand(drive) // Balance 
        );
    }
    @Override
    public void periodic() {
        queue.run();
    }

}
