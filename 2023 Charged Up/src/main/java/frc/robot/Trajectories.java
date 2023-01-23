package frc.robot;

import java.nio.file.Path;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;

public enum Trajectories {

    // Score Pre-load && Cross Line [*Bottom Start*][*Blue*](9 pts)

    // Score Pre-load && Park [*Bottom Start*][*Blue*](21 pts)

    // Cross Line and Balance Charge Station [*Middle Start*][*Blue*](15 pts)

    // Score Pre-load and second game piece [*Bottom Start*][*Blue*](13 pts)

    // Score Pre-load and second game piece and Balance Charge Station [*Bottom Start*][*Blue*](25 pts)

    // Test Autos
    MID_PARK_1("output/output/Mid_Park1.wpilib.json", Rotation2d.fromDegrees(0)),
    MID_PARK_2("output/output/Mid_Park2.wpilib.json", Rotation2d.fromDegrees(0)),
    ;

    private String path;
    private boolean lock;
    private Rotation2d rot;
    private boolean relative;

    Trajectories(String path) {
        this.path = path;
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

    public boolean lockToTarget() {
        return lock;
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