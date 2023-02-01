// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Elevator.Piece;
import frc.robot.Elevator.ScoreHeight;
import frc.robot.Lift.Heights;
import frc.robot.Lift.Pieces;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  public XboxController driverController;
  public XboxController shooterController;
  public Drivetrain drive;
  private boolean fastMode;
  private boolean slowModeToggle;
  public LED ledStrips;
  public Vision gameObjectVision;
  public String currentPiecePipeline;
  // public Elevator elevator;
  private Lift lift;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    driverController = new XboxController(0);
    shooterController = new XboxController(1);
    drive = new Drivetrain();
    ledStrips = new LED();
    gameObjectVision = new Vision(Constants.LIMELIGHT_BALL, Constants.BALL_LOCK_ERROR,
     Constants.BALL_CLOSE_ERROR, Constants.BALL_CAMERA_HEIGHT, Constants.BALL_CAMERA_ANGLE, 
     Constants.BALL_TARGET_HEIGHT, Constants.BALL_ROTATE_KP, Constants.BALL_ROTATE_KI, Constants.BALL_ROTATE_KD);
    // elevator = new Elevator();
    lift = new Lift();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    fastMode     = true;
    slowModeToggle = false;

    // elevator.reset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double rotatePower;
    double translatePower;
    double translateX;
    double translateY;
    double rotate;

    SmartDashboard.putNumber("Heading", 360 - drive.getGyroscopeRotation().getDegrees());

    gameObjectVision.updateVision();
    //
    // Read gamepad controls for drivetrain and scale control values
    //
    
    if (driverController.getXButtonPressed() && driverController.getBackButtonPressed()) {
      drive.zeroGyroscope();
    }
  
    if (driverController.getRightBumperPressed()){
      slowModeToggle = ! slowModeToggle;
    }
    fastMode = ! slowModeToggle; //&& !controlPanel.getRawButton(7); 
    

    if (fastMode) {
      rotatePower    = ConfigRun.ROTATE_POWER_FAST;
      translatePower = ConfigRun.TRANSLATE_POWER_FAST;
    }
    else {
      rotatePower    = ConfigRun.ROTATE_POWER_SLOW;
      translatePower = ConfigRun.TRANSLATE_POWER_SLOW;
    }
    
    if(driverController.getLeftBumper())
    {
      double speed = gameObjectVision.moveTowardsTarget(-0.5, -0.5);
      double turn = gameObjectVision.turnRobot(1.0);
      drive.drive(new ChassisSpeeds(speed, 0.0, turn));
    }
    else{  
      rotate = (driverController.getRightX() * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
          * rotatePower; // Right joystick
      translateX = (driverController.getLeftY() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND) * translatePower; // X
                                                                                                                          // is
                                                                                                                          // forward
                                                                                                                          // Direction,
                                                                                                                          // Forward
                                                                                                                          // on
                                                                                                                          // Joystick
                                                                                                                          // is
                                                                                                                          // Y
      translateY = (driverController.getLeftX() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND) * translatePower;

      //
      // Normal teleop drive
      //
      
      drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-joystickDeadband(translateX), -joystickDeadband(translateY),
          -joystickDeadband(rotate), drive.getGyroscopeRotation()));
    } // Inverted due to Robot Directions being the
                                                                    // opposite of controller directions
    
    drive.getCurrentPos();

    if (shooterController.getXButtonPressed()){
      currentPiecePipeline = "CUBE";
      NetworkTableInstance.getDefault().getTable("limelight-ball").getEntry("pipeline").setNumber(Constants.CUBE_PIPELINE);
      ledStrips.setFullPurple();
    }
    
    if (shooterController.getYButtonPressed()){
      currentPiecePipeline = "CONE";
      NetworkTableInstance.getDefault().getTable("limelight-ball").getEntry("pipeline").setNumber(Constants.CONE_PIPELINE);
      ledStrips.setFullYellow();
    }

    // TEST PLAN 1

    lift.testPlan1(driverController.getLeftY());

    // TEST PLAN 2

    // lift.testPlan2();

    // // TEST PLAN 3

    // if (driverController.getLeftBumper()) {
    //   lift.setPieceMode(Pieces.CONE);
    // } else if (driverController.getRightBumper()) {
    //   lift.setPieceMode(Pieces.CUBE);
    // }

    // if (driverController.getAButton()) {
    //   lift.testPlan3(Heights.INTAKE);
    // } else if (driverController.getBButton()) {
    //   lift.testPlan3(Heights.MID);
    // } else if (driverController.getYButton()) {
    //   lift.testPlan3(Heights.HIGH);
    // } else {
    //   lift.testPlan3(Heights.STOWED);
    // }

    // // TEST PLAN 4

    // if (driverController.getLeftBumper()) {
    //   lift.setPieceMode(Pieces.CONE);
    // } else if (driverController.getRightBumper()) {
    //   lift.setPieceMode(Pieces.CUBE);
    // }

    // if (driverController.getAButton()) {
    //   lift.testPlan3(Heights.INTAKE);
    // } else if (driverController.getBButton()) {
    //   lift.testPlan3(Heights.MID);
    // } else if (driverController.getYButton()) {
    //   lift.testPlan3(Heights.HIGH);
    // } else {
    //   lift.testPlan3(Heights.STOWED);
    // }

    // ========================================

    // if (shooterController.getLeftBumper()) {
    //   elevator.setPiece(Piece.CUBE);
    // } else if (shooterController.getRightBumper()) {
    //   elevator.setPiece(Piece.CONE);
    // }

    // if (shooterController.getAButton()) {
    //   elevator.liftTo(ScoreHeight.LOW);
    // } else if (shooterController.getXButton()) {
    //   elevator.liftTo(ScoreHeight.MID);
    // } else if (shooterController.getYButton()) {
    //   elevator.liftTo(ScoreHeight.HIGH);
    // } else {
    //   elevator.liftTo(ScoreHeight.STOWED);
    // }
  } 


  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

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
      return inputJoystick;
    }
  }
}
