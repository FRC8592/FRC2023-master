package frc.robot.autonomous;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.autonomous.trajectory.SwerveTrajectory;

// Adjustments made based on simulation results
public enum AutonomousPositions {

    // BLUE SIDE

    GRID_A(1.664 + 0.6, 5.013 - 0.05),
    GRID_B(1.664 + 0.6, 4.448),
    GRID_C(1.664 + 0.6, 3.876),
    GRID_D(1.664 + 0.6, 3.296),
    GRID_E(1.664 + 0.6, 2.729),
    GRID_F(1.664 + 0.6, 2.136),
    GRID_G(1.664 + 0.6, 1.579 - 0.1),
    GRID_H(1.664 + 0.6, 1.042),
    GRID_I(1.664 + 0.6, 0.384 + 0.5),
    
    B_CABLE_COVER(5.292, 1.999),
    B_MIDDLE(5.292, 2.682 + 0.2),
    B_LOADING_ZONE(5.292, 3.523),

    // Positions all mixed up for intermediary
    INTERMEDIARY_LOADING_ZONE(4.985, 4.627 + 0.5), 
    INTERMEDIARY_CABLE_COVER(4.985, 0.737 + 0.5),
    INTERMEDIARY_BALANCE(4.985, 2.727 + 0.5),

    GAME_PIECE_1(7.169 - 0.5, 4.615 + 0.4),
    GAME_PIECE_2(7.169 - 0.5, 3.398 + 0.4), // Same as GP3 in document
    GAME_PIECE_3(7.169 - 0.5, 2.138 + 0.3), // Same as GP2 in document
    GAME_PIECE_4(7.169 - 0.5, 0.964 + 0.4),
    ;

    private Translation2d translation;
    AutonomousPositions(double xPos, double yPos) {
        translation = new Translation2d(xPos, yPos);
    }

    public Translation2d getTranslation() {
        return translation;
    }

    public Pose2d getPose() {
        return new Pose2d(translation, new Rotation2d());
    }

    public SwerveTrajectory generateTrajectoryFromPoints(AutonomousPositions start, AutonomousPositions end, TrajectoryConfig config) {
        Trajectory traj = TrajectoryGenerator.generateTrajectory(
            start.getPose(), 
            new ArrayList<>(), 
            end.getPose(),
            config
        );

        return new SwerveTrajectory(traj);
    }
}
