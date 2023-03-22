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

public class CableCoverPreloadLowMobilityAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);

    private SwerveTrajectory G_TO_Icc = generate(
        config,
        GRID_G.getPose(),
        INTERMEDIARY_CABLE_COVER.translate(-2.0, 0.5),
        INTERMEDIARY_CABLE_COVER.translate(1.0, 0.5)
    );
    
    @Override
    public void initialize() {
        queue = new CommandQueue(
            new ScoreCommand(intake), // Score preload low
            new FollowerCommand(drive, G_TO_Icc) // Move out community
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}
