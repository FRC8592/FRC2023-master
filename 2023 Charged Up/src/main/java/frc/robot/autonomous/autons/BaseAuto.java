package frc.robot.autonomous.autons;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Drivetrain;
import frc.robot.Elevator;
import frc.robot.Intake;
import frc.robot.Robot;
import frc.robot.Vision;
import frc.robot.commands.CommandQueue;

public abstract class BaseAuto {
    protected Drivetrain drive;
    protected Elevator elevator;
    protected Intake intake;
    protected Vision vision;
    protected CommandQueue queue;

    protected double scoreTime = 1; // Remove once actual intake/outtake on the robot

    /**
     * Add all running subsystems for use for all autonomous routines
     * @param pDrive {@code drivetrain} object
     */
    public void addModules(Drivetrain pDrive, Elevator pLift, Intake pIntake, Vision pVision) {
        drive = pDrive;
        elevator = pLift;
        intake = pIntake;
        vision = pVision;
    }

    /**
     * Sets the robot's simulation start {@code Pose2d}
     */
    public void setInitialSimulationPose() {
        Robot.FIELD.setRobotPose(queue.getStartPose());
    }

    /**
     * @return starting {@code Pose2d} based on activated trajectories
     */
    public Pose2d getStartPose() {
        return queue.getStartPose();
    }

    public void addDelay(double seconds) {
        queue.addDelay(seconds);
    }

    public abstract void initialize();
    public abstract void periodic();
}