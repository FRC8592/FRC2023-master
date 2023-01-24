// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.AutonomousSelector;
import frc.robot.autonomous.BaseAuto;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public XboxController driverController;
  public XboxController shooterController;

  public Drivetrain drive;
  public LED ledStrips;

  private boolean fastMode;
  private boolean slowModeToggle;

  private AutonomousSelector selector;
  private BaseAuto selectedAutonomous;

  public static Field2d FIELD = new Field2d();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    driverController = new XboxController(0);
    shooterController = new XboxController(1);
    drive = new Drivetrain();
    ledStrips = new LED();

    selector = new AutonomousSelector();
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
    SmartDashboard.putData(FIELD);
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
    selectedAutonomous = selector.getSelectedAutonomous();
    selectedAutonomous.addModules(drive); // ADD EACH SUBSYSTEM ONCE FINISHED
    selectedAutonomous.initialize();
    if (!isReal()) {
      selectedAutonomous.setInitialSimulationPose();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    selectedAutonomous.periodic();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    fastMode     = true;
    slowModeToggle = false;

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

    //
    // Read gamepad controls for drivetrain and scale control values
    //
  
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
        -joystickDeadband(rotate), drive.getGyroscopeRotation())); // Inverted due to Robot Directions being the
                                                                    // opposite of controller directions
    
    drive.getCurrentPos();

    if (shooterController.getXButtonPressed()){
      ledStrips.setPurple();
    }

    if (shooterController.getYButtonPressed()){
      ledStrips.setYellow();
    }
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
