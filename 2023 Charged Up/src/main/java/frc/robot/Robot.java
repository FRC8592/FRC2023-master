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
  public Vision substationVision;
  public String currentPiecePipeline;
  private Elevator elevator;
  private Intake intake;
  public FRCLogger logger;
  public PIDController turnPID;
  public PIDController driveToPID;
  public PIDController strafePID;
  public boolean wasZeroed = false;
  private boolean coneVision = true;
  public Power power;
  private boolean angleTapBool = false;
  public BeamSensor cubeBeamSensor;
  public SmoothingFilter smoothingFilter;



  private double currentWrist = Constants.WRIST_INTAKE_ROTATIONS;

  private BaseAuto selectedAuto;
  private AutonomousSelector selector;
  public Autopark autoPark;
  private Timer timer = new Timer();

  private boolean isPartyMode = false;

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
    cubeBeamSensor = new BeamSensor(Constants.BEAM_BREAK_CUBE_ID);
    gameObjectVision = new Vision(Constants.LIMELIGHT_VISION, Constants.DRIVE_TO_LOCK_ERROR,
     Constants.DRIVE_TO_CLOSE_ERROR, Constants.DRIVE_TO_CAMERA_HEIGHT, Constants.DRIVE_TO_CAMERA_ANGLE, 
     Constants.DRIVE_TO_TARGET_HEIGHT, logger);
     substationVision = new Vision(Constants.LIMELIGHT_REAR, Constants.SUBSTATION_OFFSET,
     Constants.DRIVE_TO_CLOSE_ERROR, Constants.DRIVE_TO_CAMERA_HEIGHT, Constants.DRIVE_TO_CAMERA_ANGLE, 
     Constants.DRIVE_TO_TARGET_HEIGHT, logger);
    turnPID = new PIDController(Constants.TURN_TO_ROTATE_KP, Constants.TURN_TO_ROTATE_KI, Constants.TURN_TO_ROTATE_KD);
    driveToPID = new PIDController(Constants.DRIVE_TO_ROTATE_KP, Constants.DRIVE_TO_ROTATE_KI, Constants.DRIVE_TO_ROTATE_KD);
    ledStrips = new LED(power, gameObjectVision);
    strafePID = new PIDController(-0.05, 0, 0);
    elevator = new Elevator();
    intake = new Intake();
    // intake.reset();
    // lift.reset();
    driveScaler = new DriveScaler();

    smoothingFilter = new SmoothingFilter(1, 1, 1); //5, 5, 1

    // SmartDashboard.putData(FIELD);
    selector = new AutonomousSelector();
    
    
    // SmartDashboard.putNumber("Command Counter", 0);
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

    // SmartDashboard.putNumber("Desired Scale", driveScaler.scale(0.5));
    
    currentWrist = Constants.WRIST_INTAKE_TELEOP_ROTATIONS;
    intake.stopRoller();
    intake.haltWrist();
    elevator.set(Heights.STALL);

  //   lastXVelocity = 0;
  //  lastYVelocity = 0;

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
    substationVision.updateVision();
    elevator.update();
    // SmartDashboard.putNumber("Current Wrist", currentWrist);
    // SmartDashboard.putNumber("Roller Output Current", intake.rollerMotor.getOutputCurrent());
    logger.log(this, "RobotYaw", drive.getGyroscopeRotation());

    /*
     * Controls:
     * 
     * - Driver:
     *  - [Left Joystick X]: Drivetrain left/right
     *  - [Left Joystick Y]: Drivetrain forward/back
     *  - [Right Joysick X]: Drivetrain turn
     *  - [Right Joysick Y]: N/A
     *  - [Left Bumper]: Prime elevator
     *  - [Right Bumper]: Slow mode
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
     * 
     * - Additional Programmer Notes:
     *  - Possibly make it so that when a certain button is held the robot switches to robot-centric for manually lining up using a camera
     *  - Negative left trigger is equal to positive right trigger axis on some controllers
     *
     * - Additional Driver Notes:
     *  - Going to a pre-set elevator position automatically sets the pivot to the corresponding tilt
     *  - Make sure to turn on the robot and disable the robot with the all mechanisms back to starting configuration
     *  - Slow mode was changed from a toggle to a hold based on driver preference
     *  - Activating cone/cube mode works for both intake and scoring target-lock (*MIGHT CHANGE*)
     */

    // ========================== \\
    // ======= Drivetrain ======= \\
    // ========================== \\

    boolean shouldBalance = false;
    if (driverController.getStartButton()){
      shouldBalance = true;
    }else{
      shouldBalance = false;
    }

    if (driverController.getBackButton()) {
      drive.zeroGyroscope();
    }
    
    if (driverController.getAButtonPressed()) {
      if (isPartyMode){
        ledStrips.set(LEDMode.ATTENTION);
      }
      isPartyMode = !isPartyMode;
    }

    if (isPartyMode) {
      ledStrips.set(LEDMode.PARTY);
    } else {
      if (driverController.getYButton()) {
        NetworkTableInstance.getDefault().getTable("limelight-vision").getEntry("pipeline").setNumber(Constants.CONE_PIPELINE);
        ledStrips.set(LEDMode.CONE);
      } else if (driverController.getXButton()) {
        NetworkTableInstance.getDefault().getTable("limelight-vision").getEntry("pipeline").setNumber(Constants.CUBE_PIPELINE);
        ledStrips.set(LEDMode.CUBE);
      }
    } 
    

    // double pipeline = NetworkTableInstance.getDefault().getTable("limelight-vision").getEntry("pipeline").getDouble(10.0d);
    // SmartDashboard.putNumber("Current Pipeline", pipeline);

    if (driverController.getRightBumper()) {
      translatePower = ConfigRun.TRANSLATE_POWER_SLOW;
      rotatePower = ConfigRun.ROTATE_POWER_SLOW;
    } else {
      translatePower = ConfigRun.TRANSLATE_POWER_FAST;
      rotatePower = ConfigRun.ROTATE_POWER_FAST;
    }

    double translateXScaled = driveScaler.scale(-joystickDeadband(driverController.getLeftY()));
    double translateYScaled = driveScaler.scale(-joystickDeadband(driverController.getLeftX()));
    double rotateScaled = driveScaler.scale(joystickDeadband(driverController.getRightX())); 

    rotate = rotateScaled * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
      * rotatePower;
    
    
    translateX = translateXScaled * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * translatePower;          
    translateY = translateYScaled * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * translatePower;



    
    // double velX = driveScaler.slewFilter(lastXVelocity, driveScaler.scale(-joystickDeadband(translateX)), 4.5);
    // double velY = driveScaler.slewFilter(lastYVelocity, driveScaler.scale(-joystickDeadband(translateY)), 4.5);

    ChassisSpeeds smoothedRobotRelative = smoothingFilter.smooth(new ChassisSpeeds(translateX, translateY, 0));
    

    // SmartDashboard.putNumber("SmoothedJoystickX", smoothedRobotRelative.vxMetersPerSecond);
    // SmartDashboard.putNumber("SmoothedJoystickY", smoothedRobotRelative.vyMetersPerSecond);
    // SmartDashboard.putNumber("SmoothedJoystickRotate", smoothedRobotRelative.omegaRadiansPerSecond);

    driveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(      
      smoothedRobotRelative.vxMetersPerSecond, 
      smoothedRobotRelative.vyMetersPerSecond,
      rotate
    ), drive.getGyroscopeRotation());

    // lastXVelocity = translateX;
    // lastYVelocity = translateY;

    if (driverController.getStartButton()) { // Autobalance
      autoPark.balance(drive);
    } else if (driverController.getLeftTriggerAxis() >= 0.1) { // Turn to grid
      driveSpeeds = new ChassisSpeeds(
        driveSpeeds.vxMetersPerSecond,
        driveSpeeds.vyMetersPerSecond,
        drive.turnToAngle(0)
      );
    } else if (driverController.getRightTriggerAxis() >= 0.1 || driverController.getLeftTriggerAxis() <= -0.1) { // Track scoring grid
      double rotation = DriverStation.getAlliance() == Alliance.Red ? drive.turnToAngle(270) : drive.turnToAngle(90);
      if (substationVision.targetValid){
        if (substationVision.processedDx > Constants.SUBSTATION_OFFSET + Constants.SUBSTATION_ACCEPTANCE_RADIUS){
          ledStrips.set(LEDMode.FAR);
        }else if (substationVision.processedDx < Constants.SUBSTATION_OFFSET - Constants.SUBSTATION_ACCEPTANCE_RADIUS){
          ledStrips.set(LEDMode.CLOSE);
        } else {
          ledStrips.set(LEDMode.LOCKED);
        }

        double strafeSpeed = substationVision.lockTargetSpeed(
          0,
          strafePID,
          "tx"
,          1.0,
          Constants.SUBSTATION_OFFSET
        );

        driveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          drive.getYaw() >= 0 ? -strafeSpeed : strafeSpeed,
          -driveScaler.scale(-joystickDeadband(translateY)), 
          // -translateY * 0.3,
          rotation, 
          drive.getGyroscopeRotation());
      } else {
        driveSpeeds = new ChassisSpeeds(
          driveSpeeds.vxMetersPerSecond,
          driveSpeeds.vyMetersPerSecond,
          DriverStation.getAlliance() == Alliance.Red ? drive.turnToAngle(270) : drive.turnToAngle(90)
        ); 
      }
    } else { // Normal drive
      // driveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      //   driveScaler.scale(joystickDeadband(translateX)),
      //   driveScaler.scale(joystickDeadband(translateY)),
      //   joystickDeadband(rotate),
      //   drive.getGyroscopeRotation()
      // );
      // if (driverController.getPOV() != -1){
      //   drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-joystickDeadband(translateX), -joystickDeadband(translateY),
      //       drive.turnToAngle(driverController.getPOV()), drive.getGyroscopeRotation()));
      // }
      double turn;
      switch(driverController.getPOV()) {
        case -1:
          turn = driveSpeeds.omegaRadiansPerSecond;
          break;
        default:
          turn = drive.turnToAngle(180 - driverController.getPOV());
          break;
      }

      driveSpeeds = new ChassisSpeeds(
        driveSpeeds.vxMetersPerSecond, 
        driveSpeeds.vyMetersPerSecond,
        turn
      );
      // if (driverController.getPOV() != -1) {
        
      // } else {
      //   driveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      //     new ChassisSpeeds(      
      //       smoothedRobotRelative.vxMetersPerSecond, 
      //       smoothedRobotRelative.vyMetersPerSecond,
      //       rotate
      //     ), 
      //     drive.getGyroscopeRotation()
      //   );
      // }
    }

    if (driverController.getBButton()) { // Wheels locked
      drive.setWheelLock();
    } else if (shouldBalance){
      autoPark.balance(drive);
    } else {
      drive.drive(driveSpeeds);
    }

    // ===================== \\
    // ======= Wrist ======= \\
    // ===================== \\

    // NOTE - Left and right triggers are on the same axis in some controllers, so left trigger being negative is the same as right trigger being positive
    
    if (operatorController.getLeftTriggerAxis() >= 0.1) {
      intake.setWrist(currentWrist);
      intake.coneIntakeRoller();
        if (intake.rollerMotor.getOutputCurrent() >= Constants.ROLLER_CUBE_INTAKE_CURRENT_THRESHOLD){
          ledStrips.set(LEDMode.LOCKED);
        }else {
          ledStrips.set(LEDMode.ATTENTION);
        }
    } else if (operatorController.getLeftBumper()){
      intake.setWrist(0.0);
      intake.spinRollers(0.075);
      elevator.set(Heights.PRIME);
    } else if (operatorController.getRightTriggerAxis() >= 0.1 || operatorController.getLeftTriggerAxis() <= -0.1){
      intake.outtakeRoller();
    } else if (operatorController.getRightBumper()) {
      intake.setWrist(Constants.WRIST_INTAKE_ROTATIONS / 3);
    } else if (operatorController.getBackButtonReleased() || driverController.getLeftBumperReleased()){
      intake.setWrist(0.0);
    }else if (operatorController.getBackButton() || driverController.getLeftBumper()) {
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
          } else if (operatorController.getBButton()) {
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
          // else {
          //   elevator.set(Heights.STALL);
          // }
        }
        if (operatorController.getPOV() == 90) {
          intake.intakeRoller();
        } else if (operatorController.getPOV() == 270) {
          intake.outtakeRoller();
        } else {
          intake.stopRoller();
        }
    }

    // ======================= \\
    // ======= Rollers ======= \\
    // ======================= \\

    // if (operatorController.getLeftTriggerAxis() >= 0.1) { // Run rollers
    //   intake.intakeRoller();
    // } else if (operatorController.getRightTriggerAxis() >= 0.1 || operatorController.getLeftTriggerAxis() <= -0.1) { // Score game piece
    //   intake.scoreRoller();
    // } else if (operatorController.getRightBumper()) { // Outtake game piece
    //   intake.outtakeRoller();
    // } else { // Stop rollers
    //   intake.stopRoller();
    // }

    // ======================== \\
    // ======= Elevator ======= \\
    // ======================== \\

    // if (operatorController.getAButton()) { // Stowed height
    //   elevator.set(Heights.STOWED);
    // } else if (operatorController.getXButton()) { // Mid height
    //   elevator.set(Heights.MID);
    // } else if (operatorController.getYButton()) { // High height
    //   elevator.set(Heights.HIGH);
    // } else if (driverController.getLeftBumper()) { // Prime
    //   elevator.set(Heights.PRIME);
    // } else { // Stall at current height
    //   elevator.set(Heights.STALL);
    // }
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
    ledStrips.set(LEDMode.ATTENTION);
    elevator.writeToSmartDashboard();
    // SmartDashboard.putBoolean("Cube Beam Broken?", cubeBeamSensor.isBroken());


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
    NetworkTableInstance.getDefault().getTable("limelight-vision").getEntry("pipeline").setNumber(Constants.CUBE_PIPELINE);
    
  }

  public void testPeriodic() {
    double translatePower;
    double translateX;
    double translateY;
    double rotate;
    double rotateToAngle;
    translatePower = ConfigRun.TRANSLATE_POWER_SLOW;
    double rotatePower = ConfigRun.ROTATE_POWER_SLOW;

    ChassisSpeeds driveSpeeds = new ChassisSpeeds();

    drive.getCurrentPos();
    gameObjectVision.updateVision();

    double translateXScaled = driveScaler.scale(-joystickDeadband(driverController.getLeftY()));
    double translateYScaled = driveScaler.scale(-joystickDeadband(driverController.getLeftX()));
    double rotateScaled = driveScaler.scale(joystickDeadband(driverController.getRightX())); 

    rotate = rotateScaled * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * rotatePower;
    
    translateX = translateXScaled * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * translatePower;          
    translateY = translateYScaled * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * translatePower;
    if (driverController.getBButton()) {
    
      double rotateSpeed = gameObjectVision.lockTargetSpeed(0, turnPID, "tx", Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 0);
      rotate = rotateSpeed;

      double driveToSpeed = gameObjectVision.lockTargetSpeed(0, strafePID, "ty", 1.0, 20); // 20 means its sorta close
      translateY = driveToSpeed; // go forwards at driveToSpeed towards the target
      SmartDashboard.putNumber("pid based forward vel", driveToSpeed);
    }
    ChassisSpeeds smoothedRobotRelative = smoothingFilter.smooth(new ChassisSpeeds(translateX, translateY, 0));
    driveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(      
      smoothedRobotRelative.vxMetersPerSecond, 
      smoothedRobotRelative.vyMetersPerSecond,
      rotate
    ), drive.getGyroscopeRotation());
    drive.drive(driveSpeeds);
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
      return inputJoystick;
    }
  }
}
