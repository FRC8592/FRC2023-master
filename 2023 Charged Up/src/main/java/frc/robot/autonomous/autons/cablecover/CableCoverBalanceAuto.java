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

public class CableCoverBalanceAuto extends BaseAuto {
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
            new FollowerCommand(drive, G_TO_BM), // Move to charging station
            new AutobalanceCommand(drive) // Balance
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
