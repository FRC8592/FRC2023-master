package frc.robot.autonomous.autons.other;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.autonomous.autons.BaseAuto;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.FollowerCommand;
import static frc.robot.autonomous.AutonomousPositions.*;

public class MoveOutCommunityAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);

    private SwerveTrajectory MOVE_OUT = generateTrajectoryFromPoints(
        config,
        GRID_A.getPose(),
        GRID_A.translate(4, 0)
    );

    @Override
    public void initialize() {
        queue = new CommandQueue(
<<<<<<< HEAD:2023 Charged Up/src/main/java/frc/robot/autonomous/autons/other/MoveOutCommunityAuto.java
=======
            // new LiftCommand(elevator, Heights.PRIME), // Tilt up
            // new JointCommand( // Lift to high and score pre-load
            //     new ScoreCommand(intake), 
            //     new LiftCommand(elevator, Heights.HIGH)
            // ),
            // new LiftCommand(elevator, Heights.STOWED), // Retract the elevator
>>>>>>> main:2023 Charged Up/src/main/java/frc/robot/autonomous/autons/MoveOutCommunityAuto.java
            new FollowerCommand(drive, MOVE_OUT)
        );
    }

    @Override
    public void periodic() {
        queue.run();
    }
    
}