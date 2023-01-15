package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
/**
 * @author gavin malzahn
 * @author audrey chiang
 * FRC Season 2022
 */
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;

public class AutoDrive {

    double robotRotation;                        //angle of robot relative to global field  
    private static double hubCenterX;            //xcenter of the hub in field coordinates
    private static double hubCenterY;            //ycenter of the hub in field coordinates
    private static double hubRadius = .6;
    private boolean isGoodData;                  //can this data be used
    private double positionX;                    //position in x;
    private double positionY;                    //position in y;
    private final double KP_velocity_X = 5;
    private final double KP_velocity_Y = 5;
    private final double KD_velocity_X = 0;
    private final double KD_velocity_Y = 0;
    private final double KP_angular_velocity = 0.5;
    private final double KD_angular_velocity = 0;
    private double lastErrorX;
    private double lastErrorY;
    private double lastAngleError;
    private double targetX;
    private double targetY;
    public Timer moveTimer;
    public Timer turnTimer;
    public double lastMoveTime;
    public double lastTurnTime;
    private Drivetrain drive;

    private TrajectoryFollower mFollower;

    /**
     * 
     * @param hubCenterX
     * @param hubCenterY
     */
    public AutoDrive(double hubCenterX, double hubCenterY, Drivetrain drive) { 
        AutoDrive.hubCenterX = hubCenterX;
        AutoDrive.hubCenterY = hubCenterY;
        this.robotRotation = 0;
        AutoDrive.hubRadius = .6;
        isGoodData = false;
        targetX = 0;
        targetY = 0;
        lastErrorY = 0;
        lastErrorX = 0;
        moveTimer = new Timer();
        turnTimer = new Timer();
        moveTimer.start();
        turnTimer.start();
        lastMoveTime = moveTimer.get();
        lastTurnTime = turnTimer.get();
        this.drive = drive;
    }

    public AutoDrive(SwerveTrajectory trajectory, Drivetrain pDrive) {
        //robotRotation = trajectory.sample(0).poseMeters.getRotation().getRadians();
        isGoodData = false;
        drive = pDrive;
        targetX = 0;
        targetY = 0;
        lastErrorX = 0;
        lastErrorY = 0;
        moveTimer = new Timer();
        turnTimer = new Timer();
        lastMoveTime = moveTimer.get();
        lastTurnTime = turnTimer.get();

        mFollower = new TrajectoryFollower(trajectory);
    }

    public void followTrajectory(double pTime, boolean targetLock) {
        ChassisSpeeds speeds = mFollower.follow(drive.getCurrentPos(), pTime, targetLock);
        drive.drive(speeds);
    }

    // Needs to be called every update

    /***
     * 
     * @param robotRotation
     * @param vision
     * @param SmartDashboard 
     */
    
    public void updatePosition(double robotRotation, Vision vision){
        double targetDistance = vision.distanceToTarget();
        double targetOffsetRotation = vision.offsetAngle(); 
        double robotRotationRad = Math.toRadians(robotRotation);
        targetDistance = this.inchesToMeters(targetDistance);
        boolean testNone = false;
        if(vision.targetValid && testNone ){
            double distance2 = (targetDistance + AutoDrive.hubRadius)/Math.cos(targetOffsetRotation);
            this.positionX = -distance2 * Math.cos(robotRotationRad + targetOffsetRotation) + hubCenterX;
            this.positionY = -distance2 * Math.sin(robotRotationRad + targetOffsetRotation) + hubCenterY;
            this.isGoodData = true;
            Pose2d pose = new Pose2d(this.positionX, this.positionY, new Rotation2d());
            drive.resetPose(pose);
            
        } else {
            this.isGoodData = true;
            Pose2d pose = drive.getCurrentPos();
            this.positionX = pose.getX();
            this.positionY = pose.getY();
        }
        SmartDashboard.putNumber("Yaw value", robotRotation);
        SmartDashboard.putNumber("Position Y", this.metersToInches(positionY));
        SmartDashboard.putNumber("Position X", this.metersToInches(positionX));
        SmartDashboard.putNumber("Position Valid", isGoodData ? 1.0: 0.0);
        
    }

    /**
     * 
     * @return
     */
    public double getX(){
        return this.positionX;
    }

    public double getYaw(){
        return this.robotRotation;
    }

    /***
     * 
     * @return
     */
    public double getY(){
        return this.positionY;
    }

    /***
     * 
     * @return
     */
    public boolean isGood(){
        return this.isGoodData;
    }

    public double turnTo(double targetAngle, double currentAngle){
        double angleError = targetAngle - currentAngle;
        double angularVelocity;

        if(angleError > 180){
            angleError = angleError - 360;
        }else if(angleError < -180){
            angleError = angleError + 360;
        }


        double turnTime = turnTimer.get(); 
        double changeInAngleError = (angleError - lastAngleError)/(turnTime - lastTurnTime);
        lastAngleError = angleError;  // reset initial angle

        angularVelocity = angleError * KP_angular_velocity + KD_angular_velocity * changeInAngleError;
            angularVelocity = Math.min(angularVelocity, 0.5);
            angularVelocity = Math.max(angularVelocity, -0.5);

            return -angularVelocity;
    }

    public double[] moveTo(double setPointX, double setPointY){
    
        if(targetX != setPointX || targetY != setPointY){
            lastErrorX = 0;
            lastErrorY = 0;

        }


        double[] velocity = new double [2]; 
        velocity[0] = 0;
        velocity[1] = 0;
        
        //updatePosition(robotRotation, vision);
        if(isGoodData){
            double errorX = setPointX - this.positionX;
            double errorY = setPointY - this.positionY;

            double xtime = moveTimer.get(); 
            double changeInErrorX = (errorX - lastErrorX)/(xtime - lastMoveTime);
            lastErrorX = errorX;  // reset initial angle

            double changeInErrorY = (errorY - lastErrorY)/(xtime - lastMoveTime);
            lastErrorY = errorY;  // reset initial angle

            velocity[0] = errorX * KP_velocity_X + KD_velocity_X * changeInErrorX;
            velocity[0] = Math.min(velocity[0], 0.5);
            velocity[0] = Math.max(velocity[0], -0.5);
            velocity[1] = errorY * KP_velocity_Y + KD_velocity_Y * changeInErrorY;
            velocity[1] = Math.min(velocity[1], 0.5);
            velocity[1] = Math.max(velocity[1], -0.5);
            lastMoveTime  = xtime;        // reset initial time
            
        }
        return velocity;
    }

    public double getDistance(double x, double y){
        return Math.sqrt(Math.pow((x - getX()),2) + Math.pow((y - getY()), 2));
    }

    public double getHeading(double x, double y){
        return Math.toDegrees(Math.atan2(y - getY(), x - getX()));
    }

    public double inchesToMeters(double inch){
        return inch*0.0254;
    }

    public double metersToInches(double meters){
        return meters*39.3701;
    }
    
}
