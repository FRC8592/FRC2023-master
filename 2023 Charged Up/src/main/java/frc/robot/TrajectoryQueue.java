package frc.robot;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;

public class TrajectoryQueue {
    private Queue<SwerveTrajectory> mQueue;
    private Timer mTimer;

    public TrajectoryQueue(SwerveTrajectory ... trajectories) {
        mQueue = new LinkedList<SwerveTrajectory>();
        for (SwerveTrajectory trajectory : trajectories) {
          mQueue.add(trajectory);
        }

        mTimer = new Timer();
    }

    public void configTrajectories(TrajectoryConfig config) {
        for (SwerveTrajectory traj : mQueue) {
            Pose2d start = traj.trajectory().getInitialPose();
            Pose2d end = traj.trajectory().sample(traj.trajectory().getTotalTimeSeconds() - 0.02).poseMeters;
            List<Translation2d> interior = new ArrayList<>();
            List<State> states = traj.trajectory().getStates();
            states.remove(0);
            states.remove(states.size() - 1);
            for (State state : states) {
                interior.add(state.poseMeters.getTranslation());
            }
            traj = new SwerveTrajectory(TrajectoryGenerator.generateTrajectory(start, interior, end, config));
        }
    }

    public boolean isFinished() {
        return mQueue.size() == 0;
    }

    public boolean isTrajectoryComplete() {
        return mTimer.get() >= currentTrajectory().trajectory().getTotalTimeSeconds();
    }

    public SwerveTrajectory currentTrajectory() {
        mTimer.start();
        return mQueue.peek();
    }

    public SwerveTrajectory nextTrajectory() {
        mTimer.reset();
        mTimer.start();
        mQueue.poll();
        return mQueue.peek();
    }

    public int size() {
        return mQueue.size();
    }
}
