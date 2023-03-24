package frc.robot.autonomous.autons.middle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.autonomous.autons.BaseAuto;
import frc.robot.commands.AutobalanceCommand;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;
import static frc.robot.autonomous.AutonomousPositions.*;

public class MiddleBalanceAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(0.7, 1);
    
    private SwerveTrajectory E_TO_BM = generate(
        config.setStartVelocity(0.0).setEndVelocity(1.0),
        GRID_E.getPose(),
        GRID_E.translate(2.0, 0.0)
    ).addRotation(Rotation2d.fromDegrees(180), 0.25);

    @Override
    public void initialize() {
        queue = new CommandQueue(
            new FollowerCommand(drive, E_TO_BM), // Drive to gain mobility over charging station
            new AutobalanceCommand(drive) // Balance
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}