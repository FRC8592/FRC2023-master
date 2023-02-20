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

    // Additional points for helping with proper spline building
    COMMUNITY_LOADING_ZONE(3.284 - 0.2, 4.684),
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

    public Pose2d translate(double dx, double dy) {
        return new Pose2d(getPose().getX() + dx, getPose().getY() + dy, getPose().getRotation());
    }

    public Pose2d rotate(Rotation2d newRotation) {
        return new Pose2d(getTranslation(), getPose().getRotation().plus(newRotation));
    }

    public Pose2d translate(double dx, double dy, Rotation2d newRotation) {
        return new Pose2d(getPose().getX() + dx, getPose().getY() + dy, newRotation);
    }

    public AutonomousPositions setRotation(Rotation2d rot) {
        ref = rot;
        return this;
    }

    public static AutonomousPositions createPosition(Translation2d translate) {
        OVERRIDE.translation = translate;
        return OVERRIDE;
    }

    public static AutonomousPositions createPosition(double x, double y) {
        OVERRIDE.translation = new Translation2d(x, y);
        return OVERRIDE;
    }

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

        // return new SwerveTrajectory(traj).addRotation(endRotation.plus(red ? Rotation2d.fromDegrees(180) : new Rotation2d()));
        return new SwerveTrajectory(traj).addRotation(endRotation);
    }

    public static SwerveTrajectory generateTrajectoryFromPoints(TrajectoryConfig config, AutonomousPositions ... positions) {
        Trajectory traj = new Trajectory();
        boolean red = DriverStation.getAlliance().equals(DriverStation.Alliance.Red);
        if (red) {
            AutonomousPositions startPos = positions[0];
            AutonomousPositions endPos = positions[positions.length - 1];
            Translation2d[] nonEndPositions = new Translation2d[positions.length - 2];

            for (int i = 1; i < positions.length - 1; i++) {
                if (positions[i] != null) {
                    nonEndPositions[i-1] = new Translation2d(RED_WALL.getTranslation().getX() - positions[i].getTranslation().getX(), positions[i].getTranslation().getY());
                }
            }

            traj = TrajectoryGenerator.generateTrajectory(
                new Pose2d(
                    RED_WALL.getPose().getX() - startPos.getPose().getX(), 
                    startPos.getPose().getY(),
                    Rotation2d.fromDegrees(180)
                ), 
                List.of(
                    nonEndPositions
                ), 
                new Pose2d(
                    RED_WALL.getPose().getX() - endPos.getPose().getX(), 
                    endPos.getPose().getY(),
                    Rotation2d.fromDegrees(180)
                ), 
                config
            );
        } else {
            AutonomousPositions startPos = positions[0];
            AutonomousPositions endPos = positions[positions.length - 1];
            Translation2d[] nonEndPositions = new Translation2d[positions.length - 2];

            for (int i = 1; i < positions.length - 1; i++) {
                if (positions[i] != null) {
                    nonEndPositions[i-1] = positions[i].getPose().getTranslation();
                }
            }

            traj = TrajectoryGenerator.generateTrajectory(
                startPos.getPose(), 
                List.of(
                    nonEndPositions
                ), 
                endPos.getPose(), 
                config
            );
        }

        return new SwerveTrajectory(traj);
    }

    public static SwerveTrajectory generateTrajectoryFromPoints(TrajectoryConfig config, Pose2d ... poses) {
        Trajectory traj = new Trajectory();
        boolean red = DriverStation.getAlliance().equals(DriverStation.Alliance.Red);
        if (red) {
            Pose2d startPose = poses[0];
            Pose2d endPose = poses[poses.length - 1];
            Translation2d[] nonEndPoses = new Translation2d[poses.length - 2];

            for (int i = 1; i < poses.length - 1; i++) {
                nonEndPoses[i-1] = new Translation2d(RED_WALL.getPose().getX() - poses[i].getTranslation().getX(), poses[i].getTranslation().getY());
            }

            traj = TrajectoryGenerator.generateTrajectory(
                new Pose2d(
                    RED_WALL.getPose().getX() - startPose.getX(),
                    startPose.getY(),
                    startPose.getRotation().plus(Rotation2d.fromDegrees(180))
                ),
                List.of(nonEndPoses), 
                new Pose2d(
                    RED_WALL.getPose().getX() - endPose.getX(),
                    endPose.getY(),
                    endPose.getRotation().plus(Rotation2d.fromDegrees(180))
                ), 
                config
            );

            return new SwerveTrajectory(traj).setTrajectoryConfiguration(config);//.addRotation(Rotation2d.fromDegrees(180));
        } else {
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

    }

    // MAKE SURE TO ADD RED ONCE RED FINISHES
    public static SwerveTrajectory generateTrajectoryFromPoints(TrajectoryConfig config, Rotation2d refStartRot, Rotation2d refEndRot, AutonomousPositions ... positions) {
        Trajectory traj = new Trajectory();
        boolean red = DriverStation.getAlliance().equals(DriverStation.Alliance.Red);
        if (red) {
            AutonomousPositions startPos = positions[0];
            AutonomousPositions endPos = positions[positions.length - 1];
            Translation2d[] nonEndPositions = new Translation2d[positions.length - 2];

            for (int i = 1; i < positions.length - 1; i++) {
                if (positions[i] != null) {
                    // nonEndPositions[i-1] = RED_WALL.getTranslation().minus(positions[i].getPose().getTranslation());
                    nonEndPositions[i-1] = new Translation2d(RED_WALL.getTranslation().getX() - positions[i].getTranslation().getX(), positions[i].getTranslation().getY());
                }
            }

            traj = TrajectoryGenerator.generateTrajectory(
                new Pose2d(
                    // RED_WALL.getTranslation().minus(startPos.getTranslation()),
                    RED_WALL.getPose().getX() - startPos.getPose().getX(), 
                    startPos.getPose().getY(), 
                    refStartRot.plus(Rotation2d.fromDegrees(180))), 
                List.of(
                    nonEndPositions
                ), 
                new Pose2d(
                    RED_WALL.getPose().getX() - endPos.getPose().getX(), 
                    endPos.getPose().getY(),
                    refStartRot.plus(Rotation2d.fromDegrees(180))), 
                config
            );
        } else {
            AutonomousPositions startPos = positions[0];
            AutonomousPositions endPos = positions[positions.length - 1];
            Translation2d[] nonEndPositions = new Translation2d[positions.length - 2];

            for (int i = 1; i < positions.length - 1; i++) {
                if (positions[i] != null) {
                    nonEndPositions[i-1] = positions[i].getPose().getTranslation();
                }
            }

            traj = TrajectoryGenerator.generateTrajectory(
                new Pose2d(startPos.getTranslation(), refStartRot), 
                List.of(
                    nonEndPositions
                ), 
                new Pose2d(endPos.getTranslation(), refStartRot), 
                config
            );
        }

        return new SwerveTrajectory(traj);
    }
}