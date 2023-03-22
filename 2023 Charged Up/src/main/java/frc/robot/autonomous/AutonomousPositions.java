package frc.robot.autonomous;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;

// Adjustments made based on simulation results
public enum AutonomousPositions {
    GRID_A(1.664 + 0.6, 5.013 - 0.05),
    GRID_B(1.664 + 0.6, 4.448),
    GRID_C(1.664 + 0.6, 3.876 + 0.1),
    GRID_D(1.664 + 0.6, 3.296),
    GRID_E(1.664 + 0.6, 2.682 - 0.2 + 0.5),
    GRID_F(1.664 + 0.6, 2.136),
    GRID_G(1.664 + 0.6, 1.579 - 0.1 + 0.5),
    GRID_H(1.664 + 0.6, 1.579 - 0.1),
    GRID_I(1.664 + 0.6, 0.384 + 0.5),
    
    BALANCE_CABLE_COVER(5.292, 1.902), // Consider pushing x position out to not bump into charging station
    BALANCE_MIDDLE(5.292 - 0.75 + 0.75 + 0.5, 2.682 - 0.2 + 0.5),
    BALANCE_LOADING_ZONE(5.292, 3.523), // Consider pushing x position out to not bump into charging station

    // Positions all mixed up for intermediary
    INTERMEDIARY_LOADING_ZONE(4.985, 5.013 - 0.5), // Changed to parallel with Grid_A 
    INTERMEDIARY_CABLE_COVER(4.985, 0.884 + 0.6), // Changed to parallel with Grid_H
    INTERMEDIARY_BALANCE(4.985, 2.727 + 0.5),

    // Additional points for helping with proper spline building
    // COMMUNITY_LOADING_ZONE(3.284 - 0.2, 4.684),
    COMMUNITY_LOADING_ZONE(3.284 - 0.2, 4.0),
    COMMUNITY_CABLE_COVER(3.284 - 0.2, 0.646 + 0.5),

    GAME_PIECE_1(7.169 - 0.5, 4.615 + 0.4 - 0.4),
    GAME_PIECE_2(7.169 - 0.5, 3.398 + 0.4 - 0.2), // Same as GP3 in document; Confirm real
    GAME_PIECE_3(7.169 - 0.5, 2.138 + 0.3), // Same as GP2 in document; Confirm real
    GAME_PIECE_4(7.169 - 0.5, 0.964 + 0.4),

    TEST_1(2.0, 1.0),
    TEST_2(4.0, 1.0),

    RED_WALL(16, 0), // Middle of red alliance station wall (7.936 is middle y)

    OVERRIDE(0d, 0d),
    ;

    private Translation2d translation;
    private Rotation2d ref;
    AutonomousPositions(double xPos, double yPos) {
        translation = new Translation2d(xPos, yPos);
        ref = new Rotation2d();
    }

    public Translation2d getTranslation() {
        return translation;
    }

    public Pose2d getPose() {
        return new Pose2d(translation, ref);
    }

    public Pose2d translate(double dx, double dy) {
        return new Pose2d(getPose().getX() + dx, getPose().getY() + dy, getPose().getRotation());
    }

    public Pose2d rotate(Rotation2d newRotation) {
        return new Pose2d(getTranslation(), getPose().getRotation().plus(newRotation));
    }

    public Pose2d translate(double dx, double dy, Rotation2d newRotation) {
        return new Pose2d(getPose().getX() + dx, getPose().getY() + dy, newRotation);
    }

    public static SwerveTrajectory generate(TrajectoryConfig config, Pose2d ... poses) {
        Trajectory traj = new Trajectory();
        Pose2d startPose = poses[0];
        Pose2d endPose = poses[poses.length - 1];
        Translation2d[] nonEndPoses = new Translation2d[poses.length - 2];
        for (int i = 1; i < poses.length - 1; i++) {
            nonEndPoses[i-1] = poses[i].getTranslation();
        }

        traj = TrajectoryGenerator.generateTrajectory(
            startPose, 
            List.of(nonEndPoses), 
            endPose, 
            config
        );

        return new SwerveTrajectory(traj).setTrajectoryConfiguration(config);
    }

    public static SwerveTrajectory generate(TrajectoryConfig config, Rotation2d rotation, double turnDelay, Pose2d ... poses) {
        Trajectory traj = new Trajectory();
        Pose2d startPose = poses[0];
        Pose2d endPose = poses[poses.length - 1];
        Translation2d[] nonEndPoses = new Translation2d[poses.length - 2];
        for (int i = 1; i < poses.length - 1; i++) {
            nonEndPoses[i-1] = poses[i].getTranslation();
        }

        traj = TrajectoryGenerator.generateTrajectory(
            startPose, 
            List.of(nonEndPoses), 
            endPose, 
            config
        );

        return new SwerveTrajectory(traj).setTrajectoryConfiguration(config).addRotation(rotation, 2 * Math.PI, turnDelay);
    }
}