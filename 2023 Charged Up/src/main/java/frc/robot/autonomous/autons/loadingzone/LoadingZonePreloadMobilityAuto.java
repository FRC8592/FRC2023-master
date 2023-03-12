package frc.robot.autonomous.autons.loadingzone;

import frc.robot.Elevator.Heights;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.autonomous.autons.BaseAuto;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.JointCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.ScoreCommand;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import static frc.robot.autonomous.AutonomousPositions.*;

public class LoadingZonePreloadMobilityAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);
    
    private SwerveTrajectory C_TO_Ilz = generateTrajectoryFromPoints(
        config,
        GRID_C.getPose(),
        INTERMEDIARY_LOADING_ZONE.translate(-2.0, 0.0),
        INTERMEDIARY_LOADING_ZONE.translate(0.5, 0.0)
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
                new FollowerCommand(drive, C_TO_Ilz)
            )
        );
    }
    @Override
    public void periodic() {
        queue.run();
    }

}
