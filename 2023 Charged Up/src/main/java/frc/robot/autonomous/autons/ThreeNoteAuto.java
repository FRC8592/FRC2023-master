package frc.robot.autonomous.autons;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.autonomous.AutonomousPositions;
import frc.robot.autonomous.SwerveTrajectory;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.FollowerCommand;

public class ThreeNoteAuto extends BaseAuto {
    private TrajectoryConfig config = new TrajectoryConfig(1, 1);
    private SwerveTrajectory pathOne = AutonomousPositions.generate(config, AutonomousPositions.SUBWOOFER_MIDDLE.getPose(), AutonomousPositions.WING_NOTE_2.getPose());


    @Override
    public void initialize() {
        queue = new CommandQueue(
            new DelayCommand(1),
            new FollowerCommand(drive, pathOne),
            new DelayCommand(1)
            
        );
        // TODO Auto-generated method stub
        
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        queue.run();
        
    }
    
}
