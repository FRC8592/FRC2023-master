package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.CommandQueue;
import frc.robot.commands.TrajectoryFollowerCommand;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;

public class Autonomous{
  private Timer trajectoryTimer;
  private AutoDrive autoDrive;
  private Drivetrain mDrive;
  private TrajectoryQueue trajectoryQueue;
  private AutonomousSelector selector;

  private CommandQueue commandQueue;

  public Autonomous(Drivetrain drive) {
    trajectoryTimer = new Timer();
    mDrive = drive;
    trajectoryTimer.reset();
    selector = new AutonomousSelector();
  }

  public void initialize() {
    switch (selector.getSelectedAutonomous()) {
      case MidPark:
        commandQueue = new CommandQueue(
          new TrajectoryFollowerCommand(mDrive, Trajectories.MID_PARK_1.toTrajectory()),
          new TrajectoryFollowerCommand(mDrive, Trajectories.MID_PARK_2.toTrajectory())
        );
        break;
    }

    commandQueue.initialize();

    trajectoryTimer.reset();
    trajectoryTimer.start();

    // autoDrive = new AutoDrive(trajectoryQueue.currentTrajectory(), mDrive);
    // mDrive.resetPose(trajectoryQueue.currentTrajectory().getInitialPose());
  }

  public void periodic() 
  {
    // if (!trajectoryQueue.isFinished()) {
    //   if (trajectoryQueue.isTrajectoryComplete()) {
    //     autoDrive = new AutoDrive(trajectoryQueue.nextTrajectory(), mDrive);
    //     trajectoryTimer.reset();
    //   } else {
    //     autoDrive.followTrajectory(trajectoryTimer.get(), false);
    //   }
    // }
    commandQueue.run();
  }

  public boolean moveCloserToRing(Drivetrain drive, Vision visionRing, AutoDrive locality, double targetLockedSpeed, double targetCloseSpeed, double visionSearchSpeed, double distanceToShoot){
    boolean isInRange = false;
    if(visionRing.distanceToTarget() >= locality.metersToInches(distanceToShoot) && visionRing.isTargetValid()){
      isInRange = false;

      drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-visionRing.moveTowardsTarget(targetLockedSpeed, targetCloseSpeed), 0,
      visionRing.turnRobot(visionSearchSpeed), Rotation2d.fromDegrees(0)));
    }else{
      isInRange = true;
      this.stopDrive(drive);
    }
    return isInRange;
  }

  public boolean driveTo(double xPosition, double yPosition, boolean turnTo, AutoDrive locality, Drivetrain drive){
    boolean isDoneDrive;
    double[] velocity = locality.moveTo(xPosition, yPosition);
    double turnSpeed;
    double distance = locality.getDistance(xPosition, yPosition);

    if(turnTo){
      turnSpeed = locality.turnTo(locality.getHeading(xPosition, yPosition), drive.getGyroscopeRotation().getDegrees());
    } else {
      turnSpeed = 0;
    }

    drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(velocity[0], velocity[1], turnSpeed,drive.getGyroscopeRotation()));

    if(distance <= 0.2){
      isDoneDrive = true;
    }else{
      isDoneDrive = false;
    }

    return isDoneDrive;
  }

  public void stopDrive(Drivetrain drive){
    drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0,drive.getGyroscopeRotation()));
  }
  
  public boolean turnTo(double angle, Drivetrain drive, AutoDrive locality){
    boolean isDoneTurn = false;
    double turnSpeed = locality.turnTo(angle, drive.getAutoHeading());

    drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, turnSpeed, drive.getGyroscopeRotation()));

    if(drive.getAutoHeading() != angle){
      isDoneTurn = false;
    }else{
      isDoneTurn = true;
    }

    return isDoneTurn;
  }
}
