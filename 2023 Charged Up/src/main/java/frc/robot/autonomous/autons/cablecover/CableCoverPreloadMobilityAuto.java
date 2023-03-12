package frc.robot.autonomous.autons.cablecover;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Elevator.Heights;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.autonomous.autons.BaseAuto;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.JointCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.ScoreCommand;
import static frc.robot.autonomous.AutonomousPositions.*;

public class CableCoverPreloadMobilityAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);

    private SwerveTrajectory G_TO_Icc = generateTrajectoryFromPoints(
        config,
        GRID_G.getPose(),
        INTERMEDIARY_CABLE_COVER.translate(-2.0, 0.5),
        INTERMEDIARY_CABLE_COVER.translate(1.0, 0.5)
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
            new JointCommand( // Retract the 4-bar while exiting community 
                new LiftCommand(elevator, Heights.STOWED),
                new FollowerCommand(drive, G_TO_Icc)
            )
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
