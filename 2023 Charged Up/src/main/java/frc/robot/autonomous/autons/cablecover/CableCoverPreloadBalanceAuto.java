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

    private SwerveTrajectory G_TO_BM = generateTrajectoryFromPoints(
        config,
        GRID_G.getPose(),
        INTERMEDIARY_CABLE_COVER.translate(-2.0, -0.1),
        INTERMEDIARY_CABLE_COVER.translate(0.0, -0.1),
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
            new JointCommand( // Retract the 4-bar driving to charging station 
                new LiftCommand(elevator, Heights.STOWED),
                new FollowerCommand(drive, G_TO_BM)
            ),
            new AutobalanceCommand(drive) // Balance
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
