package frc.robot.autonomous;

import java.util.ArrayList;

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
    
    BALANCE_CABLE_COVER(5.292, 1.902, Rotation2d.fromDegrees(180)), // Consider pushing x position out to not bump into charging station
    BALANCE_MIDDLE(5.292 - 0.75 + 0.75, 2.682 - 0.2 + 0.5, Rotation2d.fromDegrees(0)),
    BALANCE_LOADING_ZONE(5.292, 3.523, Rotation2d.fromDegrees(180)), // Consider pushing x position out to not bump into charging station

    // Positions all mixed up for intermediary
    INTERMEDIARY_LOADING_ZONE(4.985, 5.013 - 0.05), // Changed to parallel with Grid_A 
    INTERMEDIARY_CABLE_COVER(4.985, 0.884), // Changed to parallel with Grid_H
    INTERMEDIARY_BALANCE(4.985, 2.727 + 0.5),

    GAME_PIECE_1(7.169 - 0.5, 4.615 + 0.4 - 0.4),
    GAME_PIECE_2(7.169 - 0.5, 3.398 + 0.4 - 0.2), // Same as GP3 in document; Confirm real
    GAME_PIECE_3(7.169 - 0.5, 2.138 + 0.3), // Same as GP2 in document; Confirm real
    GAME_PIECE_4(7.169 - 0.5, 0.964 + 0.4),

    TEST_1(2.0, 1.0),
    TEST_2(4.0, 1.0),

    RED_WALL(16, 0), // Middle of red alliance station wall (7.936 is middle y)
    ;

    private Translation2d translation;
    private Rotation2d ref;
    AutonomousPositions(double xPos, double yPos) {
        translation = new Translation2d(xPos, yPos);
        ref = new Rotation2d();
    }

    AutonomousPositions(double xPos, double yPos, Rotation2d refRotation) {
        translation = new Translation2d(xPos, yPos);
        ref = refRotation;
    }

    public Translation2d getTranslation() {
        return translation;
    }

    public Pose2d getPose() {
        return new Pose2d(translation, ref);
    }

    public AutonomousPositions setRotation(Rotation2d rot) {
        ref = rot;
        return this;
    }

    // ADD SOMETHING TO SET INTIAL AND FINAL ROTAITON FOR REFERNECE REASONS 
    public static SwerveTrajectory generateTrajectoryFromPoints(AutonomousPositions start, AutonomousPositions end, Rotation2d endRotation, TrajectoryConfig config) {
        Trajectory traj;
        boolean red = DriverStation.getAlliance().equals(DriverStation.Alliance.Red);
        if (red) {
            traj = TrajectoryGenerator.generateTrajectory(
                new Pose2d(RED_WALL.getPose().getX() - start.getPose().getX(), start.getPose().getY(), Rotation2d.fromDegrees(180 + start.getPose().getRotation().getDegrees())), 
                new ArrayList<>(), 
                new Pose2d(RED_WALL.getPose().getX() - end.getPose().getX(), end.getPose().getY(), Rotation2d.fromDegrees(180 + end.getPose().getRotation().getDegrees())),
                config
            );
        } else {
            traj = TrajectoryGenerator.generateTrajectory(
                start.getPose(), 
                new ArrayList<>(), 
                end.getPose(),
                config
            );
        }

        return new SwerveTrajectory(traj).addRotation(endRotation.plus(red ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(180)));
    }

    // ADD SOMETHING TO SET INTIAL AND FINAL ROTAITON FOR REFERNECE REASONS 
    public static SwerveTrajectory generateTrajectoryFromPoints(AutonomousPositions start, AutonomousPositions end, Rotation2d endRotation, Rotation2d startRef, Rotation2d endRef, TrajectoryConfig config) {
        Trajectory traj;
        boolean red = DriverStation.getAlliance().equals(DriverStation.Alliance.Red);
        if (red) {
            traj = TrajectoryGenerator.generateTrajectory(
                new Pose2d(RED_WALL.getPose().getX() - start.getPose().getX(), start.getPose().getY(), Rotation2d.fromDegrees(180 + start.getPose().getRotation().getDegrees() + startRef.getDegrees())), 
                new ArrayList<>(), 
                new Pose2d(RED_WALL.getPose().getX() - end.getPose().getX(), end.getPose().getY(), Rotation2d.fromDegrees(180 + end.getPose().getRotation().getDegrees() + endRef.getDegrees())),
                config
            );
        } else {
            traj = TrajectoryGenerator.generateTrajectory(
                // start.getPose(),
                new Pose2d(start.getPose().getTranslation(), Rotation2d.fromDegrees(-startRef.getDegrees())), 
                new ArrayList<>(), 
                new Pose2d(end.getPose().getTranslation(), Rotation2d.fromDegrees(-endRef.getDegrees())),
                config
            );
        }

        return new SwerveTrajectory(traj).addRotation(endRotation.plus(red ? Rotation2d.fromDegrees(180) : new Rotation2d()));
    }
}