// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Lift.Heights;
import edu.wpi.first.wpilibj.XboxController;


import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.rmi.registry.LocateRegistry;

import javax.swing.DropMode;

import com.swervedrivespecialties.swervelib.DriveController;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.LogFileUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  public XboxController driverController;
  public XboxController operatorController;
  public Drivetrain drive;
  public LED ledStrips;

  public Vision gameObjectVision;
  public String currentPiecePipeline;
  private Lift lift;
  private Intake intake;
  public FRCLogger logger;
  public PIDController turnPID;
  public PIDController strafePID;
  public boolean wasZeroed = false;
  private boolean coneVision = true;
  // public Power power;

  public Autopark autoPark;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //AdvantageKit logging code
    Logger.getInstance().recordMetadata("ProjectName", "MyProject"); // Set a metadata value
    if (isReal()) {
        Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, PowerDistribution.ModuleType.kRev); // Enables power distribution logging
    }
    else {
      if (isReal()) { // Doesn't work in simulation; redundant code to allow simulation to not crash
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.getInstance().setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
      }
    }
    Logger.getInstance().start();
    
    
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    logger = new FRCLogger(true, "CustomLogs");
    driverController = new XboxController(0);
    operatorController = new XboxController(1);
    // power = new Power();
    drive = new Drivetrain(logger);
    ledStrips = new LED();
    gameObjectVision = new Vision(Constants.LIMELIGHT_VISION, Constants.BALL_LOCK_ERROR,
     Constants.BALL_CLOSE_ERROR, Constants.BALL_CAMERA_HEIGHT, Constants.BALL_CAMERA_ANGLE, 
     Constants.BALL_TARGET_HEIGHT, logger);
    turnPID = new PIDController(Constants.BALL_ROTATE_KP, Constants.BALL_ROTATE_KI, Constants.BALL_ROTATE_KD);
    strafePID = new PIDController(-0.2, 0, 0);
    lift = new Lift();
    intake = new Intake();
    intake.reset();
    lift.reset();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    intake.writeToSmartDashboard();
    lift.writeToSmartDashboard();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    coneVision = true;
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    wasZeroed = true;
    drive.zeroGyroscope();
    drive.resetSteerAngles();
    autoPark = new Autopark();

    /*SET LIMIT ON AUTO - LIAM M */
    drive.setAutoCurrentLimit();
    autoPark = new Autopark();
  }
  
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // switch (m_autoSelected) {
    //   case kCustomAuto:
    //   // Put custom auto code here
    //   break;
    //   case kDefaultAuto:
    //   default:
    //   // Put default auto code here
    //   break;
    // }
    autoPark.balance(drive);
    lift.update();
  }
  
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    coneVision = true;
    if (!wasZeroed){
      wasZeroed = true;
      drive.zeroGyroscope();
    }
    drive.resetSteerAngles();
    /*SET LIMIT ON TELEOP - LIAM M */

    drive.setTeleopCurrentLimit();
    autoPark = new Autopark();
  }
  
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double rotatePower;
    double translatePower;
    double translateX;
    double translateY;
    double rotate;

    ChassisSpeeds driveSpeeds = new ChassisSpeeds();

    drive.getCurrentPos();
    gameObjectVision.updateVision();
    lift.update();

    /*
     * Controls:
     * - Driver:
     *  - [Left Joystick X]: drivetrain left/right
     *  - [Left Joystick Y]: drivetrain forward/back
     *  - [Right Joysick X]: Drivetrain turn
     *  - [Right Joysick Y]: N/A
     *  - [Left Bumper]: Prime elevator
     *  - [Right Bumper]: Slow mode (*changed to only when held*)
     *  - [Left Trigger]: Target-lock cone/cube
     *  - [Right Trigger]: Target-lock apriltag/retro-reflective tape
     *  - [Back btn]: Reset field-centric rotation
     *  - [Start btn]: Autopark
     *  - [A btn]: Elevator stow
     *  - [X btn]: Get human player attention (*Currently not set*)
     *  - [B btn]: Set wheels locked
     *  - [Y btn]: N/A
     *  - [DPAD Up]: Turn field-centric North (*Currently not set*)
     *  - [DPAD Down]: Turn field-centric South (*Currently not set*)
     *  - [DPAD Left]: Turn field-centric West (*Currently not set*)
     *  - [DPAD Right]: Turn field-centric East (*Currently not set*)
     * 
     * - Operator:
     *  - [Left Joystick X]: N/A
     *  - [Left Joystick Y]: Manual tilt control (*Currently not set*)
     *  - [Right Joystick X]: N/A
     *  - [Right Joystick Y]: Manual lift control (*Currently not set*)
     *  - [Left Bumper]: Stow
     *  - [Right Bumper]: Outtake
     *  - [Left Trigger]: Intake (*Auto activates wrist*)
     *  - [Right Trigger]: Score (*Auto activates wrist*)
     *  - [Back btn]: N/A
     *  - [Start btn]: N/A
     *  - [A btn]: Elevator stow (*Auto deactivates 4 bar*)
     *  - [X btn]: Elevator mid (*Auto activates 4 bar*)
     *  - [B btn]: N/A
     *  - [Y btn]: Elevator high (*Auto activates 4 bar*)
     *  - [DPAD Up]: Manual wrist up (*Currently not set*)
     *  - [DPAD Down]: Manual wrist down (*Currently not set*)
     *  - [DPAD Left]: Activate cone mode
     *  - [DPAD Right]: Activate cube mode
     */

    // ========================== \\
    // ======= Drivetrain ======= \\
    // ========================== \\

    if (driverController.getBackButton()) {
      drive.zeroGyroscope();
    }

    if (operatorController.getPOV() == 270) { // DPAD Left
      coneVision = true;
      // Set LED's to cone attention
    } else if (operatorController.getPOV() == 90) { // DPAD Right
      coneVision = false;
      // Set LED's to cube attention
    }

    if (driverController.getRightBumper()) {
      translatePower = ConfigRun.TRANSLATE_POWER_SLOW;
      rotatePower = ConfigRun.ROTATE_POWER_SLOW;
    } else {
      translatePower = ConfigRun.TRANSLATE_POWER_FAST;
      rotatePower = ConfigRun.ROTATE_POWER_FAST;
    }

    rotate = ((driverController.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
      * rotatePower;
    translateX = ((driverController.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND) * translatePower;          
    translateY = ((driverController.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND) * translatePower;

    if (driverController.getStartButton()) { // Autobalance
      autoPark.balance(drive);
    } else if (driverController.getLeftTriggerAxis() >= 0.1) { // Track game piece
      if (coneVision) {
        NetworkTableInstance.getDefault().getTable("limelight-vision").getEntry("pipeline").setNumber(Constants.CONE_PIPELINE);
      } else {
        NetworkTableInstance.getDefault().getTable("limelight-vision").getEntry("pipeline").setNumber(Constants.CUBE_PIPELINE);
      }

      // set LED to targetlock

      driveSpeeds = new ChassisSpeeds(
        driveSpeeds.vxMetersPerSecond,
        driveSpeeds.vyMetersPerSecond,
        gameObjectVision.turnRobot(
          1.0,
          turnPID,
          8.0
        )
      );
    } else if (driverController.getRightTriggerAxis() >= 0.1) { // Track scoring grid
      if (coneVision) {
        NetworkTableInstance.getDefault().getTable("limelight-vision").getEntry("pipeline").setNumber(Constants.RETROTAPE_PIPELINE);
      } else {
        NetworkTableInstance.getDefault().getTable("limelight-vision").getEntry("pipeline").setNumber(Constants.APRILTAG_PIPELINE);
      }

      // set LED to targetlock

      driveSpeeds = new ChassisSpeeds(
        driveSpeeds.vxMetersPerSecond,
        0,
        driveSpeeds.omegaRadiansPerSecond
      );
    } else { // Normal drive
      driveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        joystickDeadband(translateX), 
        joystickDeadband(translateY),
        joystickDeadband(rotate), 
        drive.getGyroscopeRotation()
      );
    }

    if (driverController.getBButton()) { // Wheels locked
      drive.setWheelLock();
    } else {
      drive.drive(driveSpeeds);
    }

    if (driverController.getXButton()) {
      // Signal to human player for attention
    }

    // ===================== \\
    // ======= Wrist ======= \\
    // ===================== \\

    if (operatorController.getLeftTriggerAxis() >= 0.1 || operatorController.getRightTriggerAxis() >= 0.1) {
      intake.enableWrist(true);
    } else if (operatorController.getLeftBumper()) {
      intake.enableWrist(false);
    }

    // ======================= \\
    // ======= Rollers ======= \\
    // ======================= \\

    if (operatorController.getLeftTriggerAxis() >= 0.1) { // Run rollers
      intake.intake();
    } else if (operatorController.getRightTriggerAxis() >= 0.1) { // Outtake game piece
      intake.score();
    } else if (operatorController.getRightBumper()) { // Score game piece
      intake.outtake();
    } else { // Stop rollers
      intake.stopRoller();
    }

    // ======================== \\
    // ======= Elevator ======= \\
    // ======================== \\

    if (operatorController.getAButton() || driverController.getAButton()) { // Stowed height
      lift.set(Heights.STOWED);
    } else if (operatorController.getXButton()) { // Mid height
      lift.set(Heights.MID);
    } else if (operatorController.getYButton()) { // High height
      lift.set(Heights.HIGH);
    } if (driverController.getLeftBumper()) { // Prime
      lift.set(Heights.PRIME);
    } else { // Stall at current height
      lift.set(Heights.STALL);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    // drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,
    // 0, drive.getGyroscopeRotation())); // Inverted due to Robot Directions being the
    // //                                                          // opposite of controller direct
    // drive.setWheelLock();
  
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  public void testPeriodic() {
    SmartDashboard.putString("Yaw", drive.getGyroscopeRotation().toString());
    SmartDashboard.putNumber("Yaw Number", drive.getYaw());

    // if (driverController.getLeftTriggerAxis() >= 0.1) {
    //   intake.enableWrist(true);
    //   if (driverController.getLeftBumper()) {
    //     intake.outtake();
    //   } else {
    //     intake.intake();
    //   }
    // } else if (driverController.getRightTriggerAxis() >= 0.1) {
    //   intake.enableWrist(false);
    // } else if (driverController.getLeftBumper()) {
    //   intake.score();
    // } else {
    //   intake.stopRoller();
    // }

    // if (driverController.getYButton()) {
    //   lift.testPlanTilt(Heights.HIGH);
    // } else if (driverController.getAButton()) {
    //   lift.testPlanTilt(Heights.STOWED);
    // } else {
    //   lift.testPlanTilt(null);
    // }

    // if (driverController.getXButton()) {
    //   lift.testPlanLift(Heights.HIGH);
    // } else if (driverController.getBButton()) {
    //   lift.testPlanLift(Heights.STOWED);
    // } else {
    //   lift.testPlanLift(null);
    // }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}


  public double joystickDeadband(double inputJoystick) {
    if (Math.abs(inputJoystick) < ConfigRun.JOYSTICK_DEADBAND) {
      return 0;
    } else {
      return inputJoystick * 0.3;
    }
  }
}
