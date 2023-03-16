package frc.robot.autonomous.autons.cablecover;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Elevator.Heights;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.autonomous.autons.BaseAuto;
import frc.robot.commands.AutobalanceCommand;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.JointCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.ScoreCommand;
import static frc.robot.autonomous.AutonomousPositions.*;

public class CableCoverPreloadBalanceAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1.8, 1);

    private SwerveTrajectory G_TO_Icc = generateTrajectoryFromPoints(
        config.setStartVelocity(0.0).setEndVelocity(1.0),
        GRID_G.getPose(),
        INTERMEDIARY_CABLE_COVER.translate(-2.25, 0.0),
        INTERMEDIARY_CABLE_COVER.translate(-1.0, 0.0),
        INTERMEDIARY_CABLE_COVER.getPose()
    );

    private SwerveTrajectory Icc_TO_BM = generateTrajectoryFromPoints(
        config.setStartVelocity(1.0).setEndVelocity(0.0),
        INTERMEDIARY_CABLE_COVER.getPose(),
        BALANCE_MIDDLE.rotate(Rotation2d.fromDegrees(180.0))
    );

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new LiftCommand(elevator, Heights.PRIME), // Tilt up
            new JointCommand( // Lift to high and score pre-load
                new ScoreCommand(intake), 
                new LiftCommand(elevator, Heights.HIGH)
            ),
            new LiftCommand(elevator, Heights.PRIME), // Retract the elevator
            new JointCommand( // Retract the 4-bar driving out community 
                new LiftCommand(elevator, Heights.STOWED),
                new FollowerCommand(drive, G_TO_Icc)
            ),
            new FollowerCommand(drive, Icc_TO_BM), // Move to Charging Station
            new AutobalanceCommand(drive) // Balance
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
