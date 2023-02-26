package frc.robot.autonomous.autons;

import frc.robot.Elevator.Heights;
import frc.robot.autonomous.AutoDrive;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.AutobalanceCommand;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.JointCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.ScoreCommand;

import static frc.robot.autonomous.AutonomousPositions.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

public class LoadingZonePreloadParkAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);
    
     @Override
    public void initialize() {
        SwerveTrajectory FIRST_LEG = generateTrajectoryFromPoints(
            config,
            GRID_A.getPose(),
            INTERMEDIARY_LOADING_ZONE.getPose(),
            BALANCE_MIDDLE.translate(0.0, 1.0, Rotation2d.fromDegrees(-90.0))
        );

        queue = new CommandQueue(
            new LiftCommand(elevator, Heights.HIGH), // Lifts to high position
            new ScoreCommand(intake), // Pull out the intake and score the pre-loaded piece
            new LiftCommand(elevator, Heights.PRIME), // Retracts the elevator and moves while it stows
            new JointCommand( // Retracts the tilt and moves towards charging station from the outside
                new LiftCommand(elevator, Heights.STOWED),
                new FollowerCommand(drive, FIRST_LEG)
            ),
            new AutobalanceCommand(drive) // Balances on charging station
        );
    }
    @Override
    public void periodic() {
        queue.run();
    }

}
