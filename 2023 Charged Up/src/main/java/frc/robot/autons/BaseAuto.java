package frc.robot.autons;

import frc.robot.Drivetrain;
import frc.robot.Robot;
import frc.robot.commands.CommandQueue;

public abstract class BaseAuto {
    protected Drivetrain drive;
    protected CommandQueue queue;

    public void addModules(Drivetrain pDrive) {
        drive = pDrive;
    }

    public void setInitialSimulationPose() {
        Robot.FIELD.setRobotPose(queue.getStartPose());
    }

    public abstract void initialize();
    public abstract void periodic();
}