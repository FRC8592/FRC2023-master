package frc.robot.autonomous.trajectory;

import java.nio.file.Path;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;

public enum Trajectories {
    MOBILITY_A("output/output/Park_B.wpilib.json"),

    THREEPIECE_A_1("output/output/3Piece_A1.wpilib.json"),
    THREEPIECE_A_2("output/output/3Piece_A2.wpilib.json"),
    THREEPIECE_A_3("output/output/3Piece_A3.wpilib.json"),
    THREEPIECE_A_4("output/output/3Piece_A4.wpilib.json"),

    TWOPIECE_PARK_A_1("output/output/2Piece_Park_A1.wpilib.json"),
    TWOPIECE_PARK_A_2("output/output/2Piece_Park_A2.wpilib.json"),
    TWOPIECE_PARK_A_3("output/output/2Piece_Park_A3.wpilib.json"),
    TWOPIECE_PARK_A_4("output/output/2Piece_Park_A4.wpilib.json"),

    PARK_B("output/output/Park_B.wpilib.json"),

    AUTO_TEST("output/output/Auto_Test.wpilib.json"),
    ;

    private String path;
    private Rotation2d rot;

    Trajectories(String path) {
        this.path = path;
        this.rot = new Rotation2d();
    }

    Trajectories(String path, Rotation2d rot) {
        this.path = path;
        this.rot = rot;
    }

    public String getName() {
        return path;
    }

    public SwerveTrajectory toTrajectory() {
        Trajectory traj = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
            traj = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (Exception e) {
            System.out.println("Unable to open " + traj + " " + Arrays.toString(e.getStackTrace()));
        }

        return new SwerveTrajectory(traj);
    }

    public static Trajectories parseEnum(SwerveTrajectory trajectory) {
        for (Trajectories traj : Trajectories.values()) {
            if (traj.toTrajectory().equals(trajectory)) {
                return traj;
            }
        }
        return null;
    }
}