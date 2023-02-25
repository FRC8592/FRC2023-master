package frc.robot.autonomous;

import java.nio.file.Path;
import java.util.Arrays;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;

/**
 *  An enum containing all WPILib Pathweaver trajectories
 *  @implNote To use {@code Pathweaver}, click {@code import project} and go to FRC2023-master\2023 Charged Up\Pathweaver
 *  @implNote Remember to follow the example trajectory if you do not understand
 */
public enum Trajectories {

    /**
     * [Explanation of what this path does]
     * @alliance {@code [Blue/Red]}
     * @start {@code [Loading Zone/Middle/Cable Cover]}
     * @time {@code [#] seconds}
     * @points {@code [#] pts}
     */
    EXAMPLE("output/[file name].wpilib.json", Rotation2d.fromDegrees(0)),
    
    ;

    private String path;
    private Rotation2d rot;

    /**
     * @param path file path for the trajectory [{@code String.java}]
     * @param rot desired ending rotation for the trajectory [{@code Rotation2d.java}]
     */
    Trajectories(String path, Rotation2d rot) {
        this.path = path;
        this.rot = rot;
    }

    /**
     * @return desired ending rotation for the trajectory [{@code Rotation2d.java}]
     */
    public Rotation2d getRotation() {
        return rot;
    }

    /**
     * @return Trajectory mapped out from the json file [{@code SwerveTrajectory.java}]
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
}