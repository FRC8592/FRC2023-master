package frc.robot;

import java.nio.file.Path;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;

public enum Trajectories {

    // Score Pre-load && Cross Line [*Bottom Start*][*Blue*](9 pts)

    BOTTOM_CROSS_LINE_1("output/BottomCrossLine1.wpilib.json", new Rotation2d(), false),

    // Score Pre-load && Park [*Bottom Start*][*Blue*](21 pts)

    BOTTOM_1_CUBE_PARK("output/Bottom1CubePark.wpilib.json", new Rotation2d(), false),


    // Cross Line and Balance Charge Station [*Middle Start*][*Blue*](15 pts)

    MID_PARK_1("output/MidPark1.wpilib.json", new Rotation2d(), false),
    MID_PARK_2("output/MidPark2.wpilib.json", new Rotation2d(), false),

    // COME UP WITH STUFF FOR THIS

    MID_2_CUBE_PARK_1("output/Mid2CubePark1.wpilib.json", new Rotation2d(), false),
    MID_2_CUBE_PARK_2("output/Mid2CubePark2.wpilib.json", new Rotation2d(), false),
    MID_2_CUBE_PARK_3("output/Mid2CubePark3.wpilib.json", new Rotation2d(), false),

    // Score Pre-load and second game piece [*Bottom Start*][*Blue*](13 pts)

    BOTTOM_2_CUBE_1("output/Bottom2Cube1.wpilib.json", new Rotation2d(), false),
    BOTTOM_2_CUBE_2("output/Bottom2Cube2.wpilib.json", new Rotation2d(), false),
    

    // Score Pre-load and second game piece and Balance Charge Station [*Bottom Start*][*Blue*](25 pts)

    BOTTOM_2_CUBE_PARK1("output/Bottom2CubePark1.wpilib.json", new Rotation2d(), false),
    BOTTOM_2_CUBE_PARK2("output/Bottom2CubePark2.wpilib.json", new Rotation2d(), false),
    BOTTOM_2_CUBE_PARK3("output/Bottom2CubePark3.wpilib.json", new Rotation2d(), false),

    // Test Trajectory for turning while moving

    TEST_TURN("output/TestTurn.wpilib.json", Rotation2d.fromDegrees(90), true),
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
    }

    Trajectories(String path, Rotation2d rot, boolean relative) {
        this.path = path;
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