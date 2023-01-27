package frc.robot.autonomous;

import frc.robot.Drivetrain;
import frc.robot.Robot;
import frc.robot.commands.CommandQueue;

public abstract class BaseAuto {
    protected Drivetrain drive;
    protected CommandQueue queue;

    protected double scoreTime = 0.25;

    public void addModules(Drivetrain pDrive) {
        drive = pDrive;
    }

    public void setInitialSimulationPose() {
        Robot.FIELD.setRobotPose(queue.getStartPose());
    }

    public abstract void initialize();
    public abstract void periodic();
}