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

    TEST_1("output/output/Test1.wpilib.json"),
    TEST_2("output/output/Test2.wpilib.json"),
    ;

    private String path;
    private Rotation2d rot;
    private boolean relative;

    Trajectories(String path) {
        this.path = path;
        this.rot = new Rotation2d();
        this.relative = true;
    }

    Trajectories(String path, Rotation2d rot) {
        this.path = path;
        this.rot = rot;
        this.relative = true;
    }

    Trajectories(String path, Rotation2d rot, boolean relative) {
        this.path = path;
        this.rot = rot;
        this.relative = relative;
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

        return new SwerveTrajectory(traj, rot, relative);
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