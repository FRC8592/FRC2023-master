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
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);

    @Override
    public void initialize() {
        SwerveTrajectory E_TO_BM = generateTrajectoryFromPoints(
            config,
            GRID_E.getPose(),
            BALANCE_MIDDLE.translate(0.25, 0.0)
        );
        
        queue = new CommandQueue(
            new LiftCommand(elevator, Heights.PRIME), // Tilt up
            new JointCommand( // Lift to high height and score pre-load piece
              new LiftCommand(elevator, Heights.HIGH),
              new ScoreCommand(intake)  
            ),
            new FollowerCommand(drive, E_TO_BM), // Drive to gain mobility over charging station
            new AutobalanceCommand(drive) // Balance
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
