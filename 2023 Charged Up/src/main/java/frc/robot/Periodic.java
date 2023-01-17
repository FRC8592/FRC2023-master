package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;

public class Periodic {
    private XboxController driverController, operatorController;
    private Drivetrain drive;
    private Vision vision;
    private LED led;

    private LEDMode mode;
    
    private boolean slowMode = false;

    public Periodic(Drivetrain pDrive, Vision pVision, LED pLED) {
        driverController = new XboxController(0);
        operatorController = new XboxController(1);

        drive = pDrive;
        vision = pVision;
        led = pLED;
    }

    public void teleopPeriodic() {
        updateVision();
        updateDrivetrain();
        updateLED();
    }

    private void updateDrivetrain() {
        double rotatePower;
        double translatePower;
        double translateX;
        double translateY;
        double rotate;

        if (driverController.getXButtonPressed() && driverController.getBackButtonPressed()) {
            drive.zeroGyroscope();
        }
        
        if (driverController.getRightBumperPressed()){
            slowMode = !slowMode;
        }
        
        if (slowMode) {
            rotatePower    = ConfigRun.ROTATE_POWER_SLOW;
            translatePower = ConfigRun.TRANSLATE_POWER_SLOW;
        }
        else {
            rotatePower    = ConfigRun.ROTATE_POWER_FAST;
            translatePower = ConfigRun.TRANSLATE_POWER_FAST;
        }

        if(driverController.getLeftBumper())
        {
            double speed = vision.moveTowardsTarget(-0.5, -0.5);
            double turn = vision.turnRobot(1.0);
            drive.drive(new ChassisSpeeds(speed, 0.0, turn));
        } else{  
            rotate = 
            (driverController.getRightX() * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND) * rotatePower;
            translateX = (driverController.getLeftY() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND) * translatePower;
            translateY = (driverController.getLeftX() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND) * translatePower;
        
            drive.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    -joystickDeadband(translateX), 
                    -joystickDeadband(translateY),
                    -joystickDeadband(rotate), drive.getGyroscopeRotation()
                )
            );
        }

        drive.getCurrentPos();
    }

    private void updateVision() {
        vision.updateVision();

        if (operatorController.getXButtonPressed()){
            NetworkTableInstance.getDefault().getTable("limelight-ball").getEntry("pipeline").setNumber(Constants.CUBE_PIPELINE);
            setLEDMode(LEDMode.CUBE);
          }
          
          if (operatorController.getYButtonPressed()){
            NetworkTableInstance.getDefault().getTable("limelight-ball").getEntry("pipeline").setNumber(Constants.CONE_PIPELINE);
            setLEDMode(LEDMode.CONE);
          }
    }

    private void updateLED() {
        switch(getLEDMode()) {
            case NONE:
                led.setOff();    
                break;
            case CONE:
                led.setFullYellow();
                break;
            case CUBE:
                led.setFullPurple();
                break;
            case CASCADING:
                led.upAndDown();
                break;
            case DISABLED: // Find a cool pattern to run while robot is on but disabled - Look at 987 2022 Robot for reference
                led.setOff();
                break;
            case TRACKING:
                led.blinkColor(1, Color.GREEN);
                break;
        }
    }

    private double joystickDeadband(double inputJoystick) {
        if (Math.abs(inputJoystick) < ConfigRun.JOYSTICK_DEADBAND) {
            return 0;
        } else {
            return inputJoystick;
        }
    }

    private void setLEDMode(LEDMode pMode) {
        mode = pMode;
    }

    private LEDMode getLEDMode() {
        return mode;
    }
}