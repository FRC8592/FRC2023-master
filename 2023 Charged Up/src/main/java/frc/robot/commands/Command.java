package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;

public abstract class Command {
    protected String tag = "DEFAULT COMMAND";

    public abstract void initialize(double pTime);
    public abstract boolean execute();
    public abstract void shutdown();

    public void setTag(String tag) {
        this.tag = tag;
    }

    public String tag() {
        return tag;
    }

    public void leak() { 
        // Experiment with commands 'leaking' into the next step of the queue
        // Not abstract yet because not confirmed
    }
    
    public Pose2d getStartPose() {
        return new Pose2d();
    }
}