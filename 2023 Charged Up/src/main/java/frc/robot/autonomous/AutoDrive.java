package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.lang.Math;

public class AutoDrive {
    double maxVelocity; 
    double maxTheta;
    double acceptanceRadius;
    
    double velocityKpX;
    double velocityKdX;
    double velocityKiX; 
    
    double velocityKpY;
    double velocityKdY;
    double velocityKiY;

    double velocityKpOmega;
    double velocityKdOmega;
    double velocityKiOmega;

    Pose2d currentGoal;
    boolean nextWaypoint;

    PIDController pidVelocityControlX;
    PIDController pidVelocityControlY;
    PIDController pidOmegaControl;
    SmoothingFilter sf = new SmoothingFilter(5, 5, 5);

    private ArrayList<Pose2d> waypoints;
    /**
     * Construct object and assign values
     * 
     * @param kPX P for X direction
     * @param kDX D for X direction
     * @param kIX I for X direction
     * @param kPY P for Y direction
     * @param kDY D for Y direction
     * @param kIY I for Y direction
     * @param kPTheta P for rotational direction
     * @param kDTheta D for rotational direction
     * @param kITheta I for rotational direction
     * @param maxV Max velocity the robot will move
     */
    public AutoDrive(double kPX, double kDX, double kIX, 
                      double kPY, double kDY, double kIY, 
                      double kPTheta, double kDTheta, double kITheta, 
                      double maxV, double maxTheta, double acceptanceRadius){
        velocityKpX = kPX;
        velocityKdX = kDX;
        velocityKiX = kIX;
        velocityKpY = kPY;
        velocityKdY = kDY;
        velocityKiY = kIY;
        velocityKpOmega = kPTheta;
        velocityKdOmega = kDTheta;
        velocityKiOmega = kITheta;  
        maxVelocity = maxV;
        this.maxTheta = maxTheta;
        pidVelocityControlX = new PIDController(kPX, kIX, kDX);
        pidVelocityControlY = new PIDController(kPY, kIY, kDY);
        pidOmegaControl = new PIDController(kPTheta, kITheta, kDTheta);
        pidOmegaControl.enableContinuousInput(-180, 180);
        this.acceptanceRadius = acceptanceRadius;
        this.waypoints = new ArrayList<Pose2d>();
    }

    /**
     * Return speed needed to move to a certain position
     * 
     * @param goal Position to move to
     * @param robot Current position
     * @return Velocities for each direction
     */
    public ChassisSpeeds moveTo(Pose2d goal, Pose2d robot) {
        double velocityX = 0;
        double velocityY = 0;
        double omega = 0;
        if(getDistance(goal, robot) >= this.acceptanceRadius) {
          velocityX = pidVelocityControlX.calculate(robot.getX(), goal.getX());
          velocityY = pidVelocityControlY.calculate(robot.getY(), goal.getY());
        
          velocityX = Math.max(Math.min(velocityX, maxVelocity), -maxVelocity);
          velocityY = Math.max(Math.min(velocityY, maxVelocity), -maxVelocity);
        }
        double errorAngle = getErrorAngle(robot, goal);
        if(Math.abs(errorAngle) >= 0.04) { //use errorAngle here
            omega = pidOmegaControl.calculate(0, errorAngle);  //this will still compute the correct error for the PID controller.
            omega = Math.max(Math.min(omega, maxTheta), -maxTheta);
        }
        ChassisSpeeds cs = new ChassisSpeeds(velocityX, velocityY, omega);
        // return sf.smooth(cs);
        return cs;
    }

    /**
     * Distance formula
     * 
     * @param a Point A
     * @param b Point B
     * @return Distance between A and B
     */
    public double getDistance(Pose2d a, Pose2d b) {
        return Math.sqrt(Math.pow(a.getX() - b.getX(), 2) + Math.pow(a.getY() - b.getY(), 2));
    }

    /**
     * Move to the next waypoint using moveTo method
     * 
     * @param robot Current position of robot
     * @return Velocities for each direction
     */
    public ChassisSpeeds moveToWayPoint(Pose2d robot) {
        if( nextWaypoint && !this.waypoints.isEmpty()){
            nextWaypoint = false;
            currentGoal = waypoints.remove(0);
        }
        nextWaypoint = (this.getDistance(robot, currentGoal) < this.acceptanceRadius && Math.abs(getErrorAngle(robot, currentGoal)) < 0.04);  //posibly add a condition for rotation
        return this.moveTo(this.currentGoal, robot);
    }
    /**
     * Add a waypoint to the movement path
     * 
     * @param point Point to move to
     */
    public void addWaypoint(Pose2d point){
        this.waypoints.add(point);
    }
    public void initWaypoints(){
        this.waypoints = new ArrayList<Pose2d>();
        this.nextWaypoint = true;
    }

    /**
     * @return whether or not the sequence of waypoints finished
     */
    public boolean finishedAllWaypoints() {
        return !nextWaypoint && waypoints.isEmpty();
    }

    public Pose2d getLastWaypoint(){
        return this.waypoints.get(this.waypoints.size() - 1);
    }


    
    public double getErrorAngle(Pose2d robot, Pose2d goal){
         /*** Computation for currect rotate errors into waypoint ****/
        double goalAngle = goal.getRotation().getRadians();
        double curAngle = robot.getRotation().getRadians();
       
        double errorAngle = 0; 
        //we only use angles between 0 and 2 PI so convert the angles to that range.
        if(goalAngle < 0){
                    goalAngle += Math.PI * 2;    
        }
        // find shortest angle difference error angle should allways be > -PI and <= PI
        errorAngle = goalAngle - curAngle;   
        if(errorAngle > Math.PI){
            errorAngle -= 2* Math.PI;
        } else if(errorAngle <= -Math.PI){
            errorAngle += 2* Math.PI;
        } 
        return errorAngle;
    }
}