package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;

public abstract class Command {
    protected String tag = "DEFAULT COMMAND";

    /**
     * Setup the command; called directly before the command is run
     */
    public abstract void initialize();

    /**
     * Ran periodically for the command
     * @return whether or not the command has finished
     */
    public abstract boolean execute();

    /**
     * Ran after the command has finished; typically for turning off or resetting a mechanism
     */
    public abstract void shutdown();

    /**
     * Sets a traceable tag for the given command; useful for cherry-picking a certain command out of the queue
     */
    public void setTag(String tag) {
        this.tag = tag;
    }

    /**
     * @return {@code String} representing the given tag for the command
     */
    public String tag() {
        return tag;
    }
    
    /**
     * Any command that changes the position or orientation of the robot should {@code @Override} this method
     * @return {@code Pose2d} representing the starting position of the robot
     */
    public Pose2d getStartPose() {
        return new Pose2d();
    }
}