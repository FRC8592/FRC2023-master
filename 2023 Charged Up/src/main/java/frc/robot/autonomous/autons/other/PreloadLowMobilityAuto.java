package frc.robot.autonomous.autons.other;

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

public class PreloadLowMobilityAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);

    private SwerveTrajectory MOVE_OUT = generateTrajectoryFromPoints(
        config,
        GRID_A.getPose(),
        GRID_A.translate(4, 0)
    );

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new ScoreCommand(intake), // Score preload low
            new JointCommand(
                new LiftCommand(elevator, Heights.STOWED), // Retract the elevator
                new FollowerCommand(drive, MOVE_OUT) // Move out of comminuty
            )
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}