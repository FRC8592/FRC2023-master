package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

import java.util.LinkedList;

import org.ejml.dense.row.decomposition.hessenberg.TridiagonalDecompositionHouseholderOrig_DDRM;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

public class Vision {
  
  //constants passed in during initilization 
  private double lockError;
  private double closeError;
  private double cameraHeight;
  private double cameraAngle;
  private double targetHeight;
  private double rotationKP;
  private double rotationKI;
  private double rotationKD;
  private double closeRotationKP;
  private double closeRotationKI;
  private double closeRotationKD;
  // Network Table entries
  private NetworkTableEntry tx;   // Angle error (x) from LimeLight camera
  private NetworkTableEntry ty;   // Angle error (y) from LimeLight camera
  private NetworkTableEntry ta;   // Target area measurement from LimeLight camera
  private NetworkTableEntry tv;   // Target valid indicator from Limelight camera
  // Shared variables
  public boolean targetValid;     // Indicate when the Limelight camera has found a target
  public boolean targetLocked;    // Indicate when the turret is centered on the target
  public boolean targetClose;     // Indicate when the robot is close to centered on the target
  public double  targetRange;     // Range from robot to target (inches)
  public Timer timer;
  public double processedDx = 0;
  private double processedDy = 0;
  //Private autoaim variables
  private double turnSpeed;
  private double lastTime  = 0;
  private double xtime     = 0;
  private double lastAngle = 0;
  private double changeInAngleError = 0;


  //constants for averaging limelight averages
  private int MIN_LOCKS = 3;
  private int STAT_SIZE = 5; 

  private LinkedList<LimelightData> previousCoordinates;

  private String limelightName;

  private double optDistance;
  private double distanceFeet;

  // Pipeline constants
  private static int BLUE_PIPELINE = 1;
  private static int RED_PIPELINE = 0;

  private final double DEG_TO_RAD = 0.0174533;
  private final double IN_TO_METERS = 0.0254;
  
  private FRCLogger logger;

  // PID for locking in X/Y
  private PIDController visionPid;
  

  /**
   * This constructor will intialize internal variables for the robot turret
   */
  public Vision(String limelightName, double lockError, double closeError,
                double cameraHeight, double cameraAngle, double targetHeight,
                FRCLogger logger) {

    // Set up networktables for limelight
    NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");

    previousCoordinates = new LinkedList<LimelightData>();

    // Establish initial values for variables we share
    targetValid   = false;
    targetLocked  = false;
    targetClose   = false;
    targetRange   = 0.0;
    timer = new Timer();
    timer.start();

    this.limelightName = limelightName;
    this.lockError     = lockError;
    this.closeError    = closeError;
    this.cameraHeight  = cameraHeight;
    this.cameraAngle   = cameraAngle;
    this.targetHeight  = targetHeight;

    // Create the PID controller
  
    
    this.logger = logger;
  }


  //
  // Reset internal variables to a benign state
  //
  public void reset() {
    targetValid   = false;
    targetLocked  = false;
    targetClose   = false;
  }


  public void updateVision(){      //method should be called continuously during autonomous and teleop
    double xError;
    double yError;
    double area;
    double totalDx = 0;
    double totalDy = 0;
    int totalValid = 0;

    // Read the Limelight data from the Network Tables
    xError      = tx.getDouble(0.0);
    yError      = ty.getDouble(0.0);
    area        = ta.getDouble(0.0);
    targetValid = (tv.getDouble(0.0) != 0); // Convert the double output to boolean

    logger.log(this, "NewestTargetValid", targetValid); //Logging up here instead of down
    //below because targetValid gets modified with the processed value in a few lines

    //generates average of limelight parameters
    previousCoordinates.add(new LimelightData(xError, yError, targetValid));
    if (previousCoordinates.size() > STAT_SIZE){
      previousCoordinates.removeFirst();
    }

    for(LimelightData data: previousCoordinates){
      if (data.ballValid == true){
        totalDx = data.dx + totalDx;
        totalDy = data.dy + totalDy;
        totalValid = totalValid +1;
      }
    }

    processedDx = (totalDx/totalValid);
    processedDy = totalDy/totalValid;
    targetValid = (totalValid >= MIN_LOCKS);

    targetRange = distanceToTarget();

    if (Math.abs(processedDx) < lockError) {    // Turret is pointing at target (or no target)
      targetLocked = targetValid;               // We are only locked when targetValid
    }           
    else{
      targetLocked = false;
    }

    if (Math.abs(processedDx) < closeError) { // Turret is close to locking
      targetClose = targetValid;              // We are only close when targetValid
    }           
    else{
      targetClose = false;
    }

    //Log all the data
    logger.log(this, "NewestDX", xError);
    logger.log(this, "NewestDY", yError);
    logger.log(this, "Area", area);
    logger.log(this, "ProcessedDX", processedDx);
    logger.log(this, "ProcessedDY", processedDx);
    logger.log(this, "ProcessedTargetValid", targetValid);
    logger.log(this, "TargetRange", targetRange);
    logger.log(this, "TargetLocked", targetLocked);
    logger.log(this, "TargetClose", targetClose);

    //post driver data to smart dashboard periodically
    //SmartDashboard.putNumber(limelightName + "/xerror in radians", Math.toRadians(xError));
    //SmartDashboard.putNumber(limelightName + "/LimelightX", xError);
    //SmartDashboard.putNumber(limelightName + "/LimelightY", yError);
    //SmartDashboard.putNumber(limelightName + "/LimelightArea", area);
    // SmartDashboard.putBoolean(limelightName + "/Target Valid", targetValid);
    //SmartDashboard.putNumber(limelightName + "/Change in Angle Error", changeInAngleError);
    //SmartDashboard.putNumber(limelightName + "/Average Y", processedDy);
    //SmartDashboard.putNumber(limelightName + "/Average X", processedDx);
    //SmartDashboard.putNumber(limelightName + "/Total Valid", totalValid);
    // SmartDashboard.putNumber(limelightName + "/Target Range", targetRange);
    // SmartDashboard.putBoolean(limelightName + "/inRange", targetRange >120 && targetRange < 265);
    // SmartDashboard.putBoolean(limelightName + "/Target Locked", targetLocked);
    //SmartDashboard.putBoolean(limelightName + "/Target Close", targetClose);
    //SmartDashboard.putNumber(limelightName + "/lockError", lockError);
  }


  public double delta(){ //gets the change in angle over time(seconds)
    xtime = timer.get(); 
    changeInAngleError = (processedDx - lastAngle)/(xtime - lastTime);
    lastAngle = processedDx;  // reset initial angle
    lastTime  = xtime;        // reset initial time
    return changeInAngleError;
  }


  /**
   * Gives distance from the robot to the target in inches
   * Com
   * ute range to target.
   * Formula taken from https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
   * @return distance in inches
   */
  public double distanceToTarget(){
    if (targetValid){
      double distanceInches = (targetHeight - cameraHeight) / Math.tan((cameraAngle + processedDy) * DEG_TO_RAD);//Equation is from limelight documentation finding distance
      return distanceInches;
    }
    return -1;
  }

  /**
   * 
   * @return angle offset in radians 
   */
  public double offsetAngle(){
    return Math.toRadians(processedDx);
  }

  public double getOffsetAngleDegrees(){
    return processedDx;
  }


  /**
   * Turn the robot based on limelight data
   * 1) If no targetValid turn in a circle until we get a targetValid indicator
   * 2) if targetValid, turns towards the target using the PID controller output for turn speed
   * 3) if targetValid and processedDx is within our "locked" criteria, stop turning
   * 
   * @return The turn speed
   */
  public double lockTargetSpeed(double defaultSpeed, PIDController targetPID, String lockToVariable, double limit, double offset){

    // Stop turning if we have locked onto the target within acceptable angular error
    // if (targetValid && targetLocked) {
    //   turnSpeed = 0;
    // }

    // Otherwise, if we have targetValid, turn towards the target using the PID controller to determine speed
    // Limit maximum speed
    if (targetValid) {
      if(lockToVariable == "tx") turnSpeed = targetPID.calculate(tx.getDouble(0.0), offset);  // Setpoint is always 0 degrees (dead center)
      else if(lockToVariable == "ty") turnSpeed = targetPID.calculate(ty.getDouble(0.0), offset);  // Setpoint is always 0 degrees (dead center)
      turnSpeed = Math.max(turnSpeed, -limit);
      turnSpeed = Math.min(turnSpeed, limit);
    }

    // If no targetValid, spin in a circle to search
    else {
      turnSpeed = defaultSpeed;    // Spin in a circle until a target is located
    }

    // SmartDashboard.putNumber(limelightName + "/Turn Speed", turnSpeed);

    logger.log(this, "Lock Speed", turnSpeed);

    return turnSpeed;
  }

  //
  // Drive towards the target.  We move forward before fully locked
  // This should probably be updated to base speed on distance from the target
  //
  public double moveTowardsTarget(double targetLockedSpeed, double targetCloseSpeed) {
    double moveSpeed = 0.0; // Default is 0 speed

    if (targetLocked == true){
      moveSpeed = targetLockedSpeed;
    }
    else if (targetClose == true){
      moveSpeed = targetCloseSpeed;
    }

    // SmartDashboard.putNumber(limelightName + "/Move Speed", moveSpeed);

    return moveSpeed;
  }


  /**
   * 
   * @return true if vision target is locked within the allowed angular error
   */
  public boolean isTargetLocked() {
    return targetLocked;
  }

  public boolean isTargetValid(){
    return targetValid;
  }


  /**
   * Local class used to store a history of limelight data to allow averaging
   */
  private class LimelightData{ 
    double dx;
    double dy;
    boolean ballValid;

    public LimelightData(double xError, double yError, boolean ballValid){
      dx = xError;
      dy = yError;
      this.ballValid = ballValid;
    }

  }
  public String getVisionName(){
    return this.limelightName;
  }
}