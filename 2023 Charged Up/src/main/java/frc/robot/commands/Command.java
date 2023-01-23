package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;

public abstract class Command {
    public abstract void initialize(double pTime);
    public abstract boolean execute();
    public abstract void shutdown();
    
    public Pose2d getStartPose() {
        return new Pose2d();
    }

    public void addExternalContingency() {
        
    }
}