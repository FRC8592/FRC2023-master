// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ConfigRun.AutoOptions;
import frc.robot.autonomous.AutoDrive;
import frc.robot.autonomous.AutonomousSelector;
import frc.robot.autonomous.autons.BaseAuto;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Lift.Heights;
import edu.wpi.first.wpilibj.XboxController;


import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  public XboxController driverController;
  public XboxController shooterController;
  public Drivetrain drive;
  private boolean fastMode;
  private boolean slowModeToggle;
  public LED ledStrips;

  public Vision gameObjectVision;
  public String currentPiecePipeline;
  private Lift lift;
  private Intake intake;
  public FRCLogger logger;
  public PIDController turnPID;
  public PIDController strafePID;
  public boolean wasZeroed = false;
  // public Power power;

  private BaseAuto selectedAuto;
  private AutonomousSelector selector;
  public Autopark autoPark;

  public static Field2d FIELD = new Field2d();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
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
      if (isReal()) { // For some reason this was throwing an exception during simulation so I moved it into here (yes I do understand the redundancy)
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.getInstance().setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
      }
    }
    Logger.getInstance().start();
    
    
    logger = new FRCLogger(true, "CustomLogs");
    driverController = new XboxController(0);
    shooterController = new XboxController(1);
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
    SmartDashboard.putData(FIELD);
    selector = new AutonomousSelector();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    intake.writeToSmartDashboard();
    lift.writeToSmartDashboard();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    selectedAuto = selector.getSelectedAutonomous();
    selectedAuto.addModules(drive, lift); // ADD EACH SUBSYSTEM ONCE FINISHED
    selectedAuto.initialize();
    selectedAuto.addDelay(selector.getDelay());
    
    if (!isReal()) {
      selectedAuto.setInitialSimulationPose();
    } else {
      drive.zeroGyroscope();
      drive.resetEncoder();
      drive.resetPose(selectedAuto.getStartPose());
    }

    // SmartDashboard.putString("Auto Selected", selectedAuto.getClass().getSimpleName());
    drive.resetSteerAngles();
    autoPark = new Autopark();
    drive.setAutoCurrentLimit();
  }
  
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    selectedAuto.periodic();
    lift.periodic();
    ledStrips.upAndDown();
    autoPark.balance(drive);
  }
  
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    fastMode = true;
    slowModeToggle = false;
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
    
    // System.out.println(driverControler.getBButton());
    SmartDashboard.putNumber("Heading", 360 - drive.getGyroscopeRotation().getDegrees());

    gameObjectVision.updateVision();
    lift.writeToSmartDashboard();

    //
    // Read gamepad controls for drivetrain and scale control values
    //
    
    if (driverController.getXButton() && driverController.getBackButton()) {
      drive.zeroGyroscope();
    }
  
  
    if (driverController.getRightBumperPressed()){
      slowModeToggle = ! slowModeToggle;
    }
    fastMode = ! slowModeToggle; //&& !controlPanel.getRawButton(7); 
    
    
    if (fastMode) {
      rotatePower = ConfigRun.ROTATE_POWER_FAST;
      translatePower = ConfigRun.TRANSLATE_POWER_FAST;
    } else {
      rotatePower = ConfigRun.ROTATE_POWER_SLOW;
      translatePower = ConfigRun.TRANSLATE_POWER_SLOW;
    }

    // X
    // is
    // forward
    // Direction,
    // Forward
    // on
    // Joystick
    // is
    // Y
    rotate = ((driverController.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
        * rotatePower; // Right joystick
    translateX = ((driverController.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND) * translatePower;          
    translateY = ((driverController.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND) * translatePower;

    //
    // Normal teleop drive
    //
    
    drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(joystickDeadband(translateX), joystickDeadband(translateY),
        joystickDeadband(rotate), drive.getGyroscopeRotation()));
                                                                  // opposite of controller directions
    
    drive.getCurrentPos();

    if (shooterController.getXButtonPressed()){
      currentPiecePipeline = "CUBE";
      NetworkTableInstance.getDefault().getTable("limelight-vision").getEntry("pipeline").setNumber(Constants.CUBE_PIPELINE);
      ledStrips.setFullPurple();
    }

    if (shooterController.getYButtonPressed()) {
      currentPiecePipeline = "CONE";
      NetworkTableInstance.getDefault().getTable("limelight-vision").getEntry("pipeline").setNumber(Constants.CONE_PIPELINE);
      ledStrips.setFullYellow();
    }
    //TODO:don't know if the buttons are already in use
    if (shooterController.getAButtonPressed()){
      currentPiecePipeline = "APRILTAG";
      NetworkTableInstance.getDefault().getTable("limelight-vision").getEntry("pipeline").setNumber(Constants.APRILTAG_PIPELINE);
      ledStrips.setFullOrange();
    }

    if (shooterController.getBButtonPressed()){
      currentPiecePipeline = "RETROTAPE";
      NetworkTableInstance.getDefault().getTable("limelight-vision").getEntry("pipeline").setNumber(Constants.RETROTAPE_PIPELINE);
      ledStrips.setFullBlue();
    }

    if (driverController.getLeftTriggerAxis() >= 0.1) {
      intake.enableWrist(true);
      if (driverController.getLeftBumper()) {
        intake.outtake();
      } else {
        intake.intake();
      }
    } else if (driverController.getRightTriggerAxis() >= 0.1) {
      intake.enableWrist(false);
    } else if (driverController.getLeftBumper()) {
      intake.score();
    } else {
      intake.stopRoller();
    }

    if (driverController.getYButton()) {
      lift.testPlanTilt(Heights.HIGH);
    } else if (driverController.getAButton()) {
      lift.testPlanTilt(Heights.STOWED);
    } else {
      lift.testPlanTilt(null);
    }

    if (driverController.getXButton()) {
      lift.testPlanLift(Heights.HIGH);
    } else if (driverController.getBButton()) {
      lift.testPlanLift(Heights.STOWED);
    } else {
      lift.testPlanLift(null);
    }

    if (driverController.getStartButton()) {
      autoPark.balance(drive);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,
        0, drive.getGyroscopeRotation())); // Inverted due to Robot Directions being the
    // // opposite of controller direct
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  public void testPeriodic() {
    SmartDashboard.putString("Yaw", drive.getGyroscopeRotation().toString());
    SmartDashboard.putNumber("Yaw Number", drive.getYaw());
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  public double joystickDeadband(double inputJoystick) {
    if (Math.abs(inputJoystick) < ConfigRun.JOYSTICK_DEADBAND) {
      return 0;
    } else {
      return inputJoystick * 0.3;
    }
  }
}
