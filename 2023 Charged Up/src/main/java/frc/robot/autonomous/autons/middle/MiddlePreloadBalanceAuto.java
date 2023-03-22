package frc.robot.autonomous.autons.middle;

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

public class MiddlePreloadBalanceAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(0.7, 1);

    @Override
    public void initialize() {
        SwerveTrajectory E_TO_BM = generate(
            config,
            GRID_E.getPose(),
            GRID_E.translate(4.0, 0.0)
        );
        
        queue = new CommandQueue(
            new LiftCommand(elevator, Heights.PRIME), // Tilt up
            new JointCommand( // Lift to high height and score pre-load piece
              new LiftCommand(elevator, Heights.HIGH),
              new ScoreCommand(intake)  
            ),
            new LiftCommand(elevator, Heights.PRIME),
            new JointCommand(
                new FollowerCommand(drive, E_TO_BM), // Drive to gain mobility over charging station
                new LiftCommand(elevator, Heights.STOWED)
            ),
            new AutobalanceCommand(drive) // Balance
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
