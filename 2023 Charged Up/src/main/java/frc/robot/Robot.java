// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LED.LEDMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.ConfigRun.AutoOptions;
import frc.robot.Elevator.Heights;
import frc.robot.autonomous.AutoDrive;
import frc.robot.autonomous.AutonomousSelector;
import frc.robot.autonomous.autons.BaseAuto;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;



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
  public XboxController operatorController;
  public Drivetrain drive;
  public LED ledStrips;

  public Vision gameObjectVision;
  public String currentPiecePipeline;
  private Elevator elevator;
  private Intake intake;
  public FRCLogger logger;
  public PIDController turnPID;
  public PIDController strafePID;
  public boolean wasZeroed = false;
  private boolean coneVision = true;
  public Power power;
  private boolean angleTapBool = false;

  private double currentWrist = Constants.WRIST_INTAKE_ROTATIONS;

  private BaseAuto selectedAuto;
  private AutonomousSelector selector;
  public Autopark autoPark;
  private Timer timer = new Timer();

  private DriveScaler driveScaler;

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
    if (true) {
        Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, PowerDistribution.ModuleType.kRev); // Enables power distribution logging
    }
    Logger.getInstance().start();
    
    
    logger = new FRCLogger(true, "CustomLogs");
    driverController = new XboxController(0);
    operatorController = new XboxController(1);
    power = new Power();
    drive = new Drivetrain(logger);
    gameObjectVision = new Vision(Constants.LIMELIGHT_VISION, Constants.BALL_LOCK_ERROR,
     Constants.BALL_CLOSE_ERROR, Constants.BALL_CAMERA_HEIGHT, Constants.BALL_CAMERA_ANGLE, 
     Constants.BALL_TARGET_HEIGHT, logger);
    turnPID = new PIDController(Constants.BALL_ROTATE_KP, Constants.BALL_ROTATE_KI, Constants.BALL_ROTATE_KD);
    ledStrips = new LED(power, gameObjectVision);
    strafePID = new PIDController(-0.05, 0, 0);
    elevator = new Elevator();
    intake = new Intake();
    // intake.reset();
    // lift.reset();
    driveScaler = new DriveScaler();
    SmartDashboard.putData(FIELD);
    selector = new AutonomousSelector();

    SmartDashboard.putNumber("Command Counter", 0);
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
    elevator.writeToSmartDashboard();
    power.powerPeriodic();
    ledStrips.updatePeriodic();
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
    selectedAuto.addModules(drive, elevator, intake, gameObjectVision); // ADD EACH SUBSYSTEM ONCE FINISHED
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
    coneVision = true;
    wasZeroed = true;
    drive.resetSteerAngles();
    autoPark = new Autopark();
    drive.setAutoCurrentLimit();
  }
  
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    selectedAuto.periodic();
    // lift.periodic();
    // ledStrips.upAndDown();
    // autoPark.balance(drive);
    elevator.update();
    gameObjectVision.updateVision();
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

    SmartDashboard.putNumber("Desired Scale", driveScaler.scale(0.5));

    intake.stopRoller();
    intake.haltWrist();
    elevator.set(Heights.STALL);
  }
  
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double rotatePower;
    double translatePower;
    double translateX;
    double translateY;
    double rotate;
    double rotateToAngle;

    ChassisSpeeds driveSpeeds = new ChassisSpeeds();

    drive.getCurrentPos();
    gameObjectVision.updateVision();
    elevator.update();
    SmartDashboard.putNumber("Current Wrist", currentWrist);

    /*
     * Controls:
     * 
     * - Driver:
     *  - [Left Joystick X]: Drivetrain field-centric left/right
     *  - [Left Joystick Y]: Drivetrain field-centric forward/back
     *  - [Right Joysick X]: Drivetrain rotate
     *  - [Right Joysick Y]: N/A
     *  - [Left Bumper]: N/A
     *  - [Right Bumper]: Slow mode
     *  - [Left Trigger]: N/A
     *  - [Right Trigger]: Target-Lock Substation
     *  - [Back btn]: Reset field-centric rotation
     *  - [Start btn]: Autopark
     *  - [A btn]: LED Party Mode
     *  - [X btn]: LED Cube Mode
     *  - [B btn]: Set wheels locked
     *  - [Y btn]: LED Cone Mode
     *  - [DPAD Up]: Turn field-centric North
     *  - [DPAD Down]: Turn field-centric South
     *  - [DPAD Left]: Turn field-centric West
     *  - [DPAD Right]: Turn field-centric East
     * 
     * - Operator:
     *  - [Left Joystick X]: N/A
     *  - [Left Joystick Y]: N/A
     *  - [Right Joystick X]: N/A
     *  - [Right Joystick Y]: N/A
     *  - [Left Bumper]: Intake from Substation
     *  - [Right Bumper]: Pull out Wrist
     *  - [Left Trigger]: Wrist out and intake
     *  - [Right Trigger]: Spin rollers reverse
     *  - [Back btn]: N/A
     *  - [Start btn]: Activate Manual Mode
     *  - [A btn]: Tilt down and Elevator Stow
     *  - [X btn]: Tilt up and Elevator Mid
     *  - [B btn]: Prime Elevator
     *  - [Y btn]: Tilt up and Elevator High
     *  - [DPAD Up + Start btn]: Trim wrist up
     *  - [DPAD Down + Start btn]: Trim wrist down
     *  - [DPAD Left]: Override roller spin reverse
     *  - [DPAD Right]: Override roller spin forward
     * 
     * - Additional Driver Notes:
     *  - Going to a pre-set elevator position automatically sets the pivot to the corresponding tilt
     *  - Make sure to turn on the robot and disable the robot with the all mechanisms back to starting configuration
     *  - Slow mode was changed from a toggle to a hold based on driver preference
     */

    // ========================== \\
    // ======= Drivetrain ======= \\
    // ========================== \\

    if (driverController.getBackButton()) {
      drive.zeroGyroscope();
    }

    if (driverController.getAButton()) {
      ledStrips.set(LEDMode.PARTY);
    } else if (driverController.getYButton()) {
      NetworkTableInstance.getDefault().getTable("limelight-vision").getEntry("pipeline").setNumber(Constants.CONE_PIPELINE);
      ledStrips.set(LEDMode.CONE);
    } else if (driverController.getXButton()) {
      NetworkTableInstance.getDefault().getTable("limelight-vision").getEntry("pipeline").setNumber(Constants.CUBE_PIPELINE);
      ledStrips.set(LEDMode.CUBE);
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

    driveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      driveScaler.scale(-joystickDeadband(translateX)), 
      driveScaler.scale(-joystickDeadband(translateY)), 
      driveScaler.scale(joystickDeadband(rotate)), 
      drive.getGyroscopeRotation()
    );

    if (driverController.getBButton()) { // Lock wheels
      drive.setWheelLock();
    } else if (driverController.getStartButton()) { // Autobalance
      autoPark.balance(drive);
    } else {
      if (driverController.getRightTriggerAxis() >= 0.1 || driverController.getLeftTriggerAxis() <= -0.1) { // Track substation
        ledStrips.set(LEDMode.TARGETLOCK);
        if (gameObjectVision.targetValid) {
          driveSpeeds = new ChassisSpeeds(
            driveSpeeds.vxMetersPerSecond,
            gameObjectVision.turnRobot(
              1.0,
              strafePID,
              8.0
            ),
            driveSpeeds.omegaRadiansPerSecond
          );
        }
      }
      
      double turn;
      switch(driverController.getPOV()) {
        case 0:
          turn = drive.turnToAngle(180.0);
          ledStrips.set(LEDMode.TARGETLOCK);
          break;
        case 90:
          turn = drive.turnToAngle(90.0);
          break;
        case 180:
          turn = drive.turnToAngle(0.0);
          break;
        case 270:
          turn = drive.turnToAngle(270.0);
          break;
        default:
          turn = driveSpeeds.omegaRadiansPerSecond;
          break;
      }

      driveSpeeds = new ChassisSpeeds(
        driveSpeeds.vxMetersPerSecond, 
        driveSpeeds.vyMetersPerSecond,
        turn
      );
    }

    if (driverController.getBButton()) {
      drive.setWheelLock();
    } else if (driverController.getStartButton()){
      autoPark.balance(drive);
    } else {
      drive.drive(driveSpeeds);
    }

    if (operatorController.getLeftTriggerAxis() >= 0.1) {
      intake.setWrist(currentWrist);
      if (operatorController.getXButton()) {
        intake.cubeIntakeRoller();
      } else if (operatorController.getYButton()) {
        intake.setWrist(Constants.WRIST_INTAKE_ROTATIONS * 2 / 3);
      } else {
        intake.coneIntakeRoller();
      }
    } else if (operatorController.getLeftBumper()){
      intake.setWrist(0.0);
      elevator.set(Heights.PRIME);
    } else if (operatorController.getRightTriggerAxis() >= 0.1 || operatorController.getLeftTriggerAxis() <= -0.1){
      intake.outtakeRoller();
    } else if (operatorController.getRightBumper()) {
      intake.setWrist(0.0);
    } else if (operatorController.getBackButton()) {
      intake.throwPiece();
    } else {
        if (operatorController.getStartButton()) {
          if (angleTapBool) {
            if (operatorController.getPOV() == -1) {
              angleTapBool = false;
            }
          } else if (operatorController.getPOV() == 0) {
            angleTapBool = true;
            currentWrist -= 0.25;
            intake.setWrist(currentWrist);
          } else if (operatorController.getPOV() == 180) {
            angleTapBool = true;
            currentWrist += 0.25;
            intake.setWrist(currentWrist);
          }
        } else {
          if (operatorController.getAButton()) {
            elevator.set(Heights.STOWED);
            intake.setWrist(0.0);
          } else if (operatorController.getBButton() || driverController.getLeftBumper()) {
            elevator.set(Heights.PRIME);
            intake.setWrist(0.0);
          } else if (operatorController.getXButton()) {
            elevator.set(Heights.MID);
            if (elevator.atTiltReference()) {
              intake.setWrist(Constants.WRIST_INTAKE_ROTATIONS);
            }
          } else if (operatorController.getYButton()) {
            elevator.set(Heights.HIGH);
            if (elevator.atTiltReference()) {
              intake.setWrist(Constants.WRIST_INTAKE_ROTATIONS);
            }
          } 
        }
        if (operatorController.getPOV() == 90) {
          intake.intakeRoller();
        } else if (operatorController.getPOV() == 270) {
          intake.outtakeRoller();
        } else {
          intake.stopRoller();
        }
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    intake.stopRoller();
    intake.haltWrist();
    elevator.set(Heights.STALL);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    ledStrips.set(LEDMode.PARTY);
    elevator.writeToSmartDashboard();
    // else if(operatorController.getBButton()){
    //     ledStrips.set(LEDMode.TARGETLOCK);
    // }
    // else if(operatorController.getXButton()){
    //     ledStrips.set(LEDMode.UP_AND_DOWN);
    // } 
    // else if (operatorController.getYButton()) {
    //   ledStrips.set(LEDMode.WAVES);
    // }
    drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,
        0, drive.getGyroscopeRotation())); // Inverted due to Robot Directions being the
          //  intake.logBeamBreaks();

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
