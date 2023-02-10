package frc.robot.autonomous.trajectory;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;

/**
 *  An enum containing all WPILib Pathweaver trajectories
 *  @implNote Make sure to have all the correct paths in {@code"src/main/deploy/output/output"} file directory
 */
public enum Trajectories {

    // ===========================================
    // One Piece Auto
    // ===========================================

    /**
     * Moves out of community from the upper position
     * @category Mobility Auto {@code 3 points}
     * @alliance Blue
     * @category Upper Start
     */
    MOBILITY_A("output/output/Park_B.wpilib.json"),

    // ===========================================
    // Three Piece Auto
    // ===========================================

    /**
     * Moves from scoring grid to first piece [Step 1]
     * @auto {@code TopThreePieceAuto.java}
     * @points {@code 19 points}
     * @alliance Blue
     * @start Top Start
     */
    THREEPIECE_A_1("output/output/3Piece_A1.wpilib.json"),

    /**
     * Moves from first game piece to scoring grid [Step 2]
     * @auto {@code TopThreePieceAuto.java}
     * @points {@code 19 points}
     * @alliance Blue
     * @start Top Start
     */
    THREEPIECE_A_2("output/output/3Piece_A2.wpilib.json"),

    /**
     * Moves from scoring grid to second game piece [Step 3]
     * @auto {@code TopThreePieceAuto.java}
     * @points {@code 19 points}
     * @alliance Blue
     * @start Top Start
     */
    THREEPIECE_A_3("output/output/3Piece_A3.wpilib.json"),

    /**
     * Moves from second game piece to scoring grid [Step 4]
     * @auto {@code TopThreePieceAuto.java}
     * @points {@code 19 points}
     * @alliance Blue
     * @start Top Start
     */
    THREEPIECE_A_4("output/output/3Piece_A4.wpilib.json"),

    // ===========================================
    // Two Piece and Park Auto
    // ===========================================

    /**
     * Moves from scoring grid to first piece [Step 1]
     * @auto {@code TopTwoPieceParkAuto.java}
     * @points {@code 27 points}
     * @alliance Blue
     * @start Top Start
     */
    TWOPIECE_PARK_A_1("output/output/2Piece_Park_A1.wpilib.json"),

    /**
     * Moves from first piece to scoring grid [Step 2]
     * @auto {@code TopTwoPieceParkAuto.java}
     * @points {@code 27 points}
     * @alliance Blue
     * @start Top Start
     */
    TWOPIECE_PARK_A_2("output/output/2Piece_Park_A2.wpilib.json"),

    /**
     * Moves from scoring grid to second piece [Step 3]
     * @auto {@code TopTwoPieceParkAuto.java}
     * @points {@code 27 points}
     * @alliance Blue
     * @start Top Start
     */
    TWOPIECE_PARK_A_3("output/output/2Piece_Park_A3.wpilib.json"),

    /**
     * Moves from second piece to charging station [Step 4]
     * @auto {@code TopTwoPieceParkAuto.java}
     * @points {@code 27 points}
     * @alliance Blue
     * @start Top Start
     */
    TWOPIECE_PARK_A_4("output/output/2Piece_Park_A4.wpilib.json"),

    // ===========================================
    // Park Auto
    // ===========================================

    /**
     * Engage charging station
     * @auto {@code MidParkAuto.java}
     * @points {@code 12 pts}
     * @alliance Blue
     * @start Middle Start
     */
     PARK_B("output/output/Park_B.wpilib.json"),
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

    /**
     * @return {@code Rotation2d} object representing the desired ending rotation
     */
    public Rotation2d getRotation() {
        return rot;
    }

    /**
     * @return {@code SwerveTrajectory} object with no turn
     */
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

    /**
     * @return {@code SwerveTrajectory} object with no turn and a set {@code TrajectoryConfig}
     */
    public SwerveTrajectory toTrajectory(TrajectoryConfig config) {
        Trajectory traj = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
            traj = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

            Pose2d startPose = traj.getInitialPose();
            Pose2d endPose = traj.sample(traj.getTotalTimeSeconds() - 0.02).poseMeters;
            List<Translation2d> interiorWaypoints = new ArrayList<>();
            for (Trajectory.State curState : traj.getStates()) {
                Translation2d translation = curState.poseMeters.getTranslation();
                interiorWaypoints.add(translation);
            }

            traj = TrajectoryGenerator.generateTrajectory(startPose, interiorWaypoints, endPose, config);
        } catch (Exception e) {
            System.out.println("Unable to open " + traj + " " + Arrays.toString(e.getStackTrace()));
        }
        return new SwerveTrajectory(traj);
    }
}