package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;

public class Autonomous{
  private Timer trajectoryTimer;
  private AutoDrive autoDrive;
  private Drivetrain mDrive;
  private TrajectoryQueue trajectoryQueue;
  private AutonomousSelector selector;

  public Autonomous(Drivetrain drive) {
    trajectoryTimer = new Timer();
    mDrive = drive;
    trajectoryTimer.reset();
    selector = new AutonomousSelector();
  }

  public void initialize() {
    switch (selector.getSelectedAutonomous()) {
      case MidPark:
        trajectoryQueue = new TrajectoryQueue(
          Trajectories.MID_PARK_1.toTrajectory(),
          Trajectories.MID_PARK_2.toTrajectory()
        );
        break;
      case Bottom2Cube:
        trajectoryQueue = new TrajectoryQueue(
          Trajectories.BOTTOM_2_CUBE_1.toTrajectory(),
          Trajectories.BOTTOM_2_CUBE_2.toTrajectory()
        );
        break;
      case Bottom2CubePark:
        trajectoryQueue = new TrajectoryQueue(
          Trajectories.BOTTOM_2_CUBE_PARK1.toTrajectory(),
          Trajectories.BOTTOM_2_CUBE_PARK2.toTrajectory(),
          Trajectories.BOTTOM_2_CUBE_PARK3.toTrajectory()
        );
        break;
      case BottomCrossLine:
        trajectoryQueue = new TrajectoryQueue(
          Trajectories.BOTTOM_CROSS_LINE_1.toTrajectory()
        );
        break;
      case Bottom1CubePark:
        trajectoryQueue = new TrajectoryQueue(
          Trajectories.BOTTOM_1_CUBE_PARK.toTrajectory()
        );
        break;
      case Mid2CubePark:
        trajectoryQueue = new TrajectoryQueue(
          Trajectories.MID_2_CUBE_PARK_1.toTrajectory(),
          Trajectories.MID_2_CUBE_PARK_2.toTrajectory(),
          Trajectories.MID_2_CUBE_PARK_3.toTrajectory()
        );
        break;
      case TestTurn:
        trajectoryQueue = new TrajectoryQueue(
          Trajectories.TEST_TURN.toTrajectory()
        );
        break;
      case TestTurnCole:
        trajectoryQueue = new TrajectoryQueue(
          Trajectories.TEST_TURN_COLE.toTrajectory()
        );
        break;
    }

    trajectoryTimer.reset();
    trajectoryTimer.start();

    autoDrive = new AutoDrive(trajectoryQueue.currentTrajectory(), mDrive);
    mDrive.resetPose(trajectoryQueue.currentTrajectory().getInitialPose());
  }

  public void periodic() 
  {
    if (!trajectoryQueue.isFinished()) {
      if (trajectoryQueue.isTrajectoryComplete()) {
        autoDrive = new AutoDrive(trajectoryQueue.nextTrajectory(), mDrive);
        trajectoryTimer.reset();
      } else {
        autoDrive.followTrajectory(trajectoryTimer.get(), false);
      }
    }
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
