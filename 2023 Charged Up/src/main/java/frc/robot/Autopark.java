package frc.robot;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Autopark {
    private Vision vision;
    private PIDController visionPID;

    public enum AutoBalanceStates {
        DRIVE_FORWARD,
        FIX_TILT,
        STOP;
    }
    
    AutoBalanceStates currentState;
    Timer timer;
    public Autopark(){
        currentState = AutoBalanceStates.FIX_TILT;
        timer = new Timer();

        visionPID = new PIDController(0.05, 0.0, 0.0);
    }

    public void enableVision(Vision vision) {
        this.vision = vision;
    }
    
    public boolean balance(Drivetrain drivetrain){
        double pitch = drivetrain.getPitch();
        // System.out.println(currentState.toString() + " " + pitch);
        
        switch (currentState){
            
            case DRIVE_FORWARD:
                //if pitched up
                timer.start();
                if(Math.abs(pitch) >= Constants.LEVEL_PITCH && timer.get() >= 2.0) {
                    currentState = AutoBalanceStates.FIX_TILT; 
                }
                else {
                    drivetrain.drive(new ChassisSpeeds(-0.7, 0, 0)); //the slower the better
                    SmartDashboard.putNumber("Movement speed", 0.7);
                }
                break;
            
            case FIX_TILT:
                
                if(Math.abs(pitch) <= Constants.LEVEL_PITCH) {
                    currentState = AutoBalanceStates.STOP; 
                }
                else{
                    ChassisSpeeds desired = new ChassisSpeeds(-(pitch * Constants.PITCH_MULTIPLIER), 0, 0);
                    if (vision != null) {
                        double vyVision = vision.turnRobot( 0,   visionPID,   1.0, 0.0);
                        desired = new ChassisSpeeds(
                            desired.vxMetersPerSecond,
                            vyVision,
                            desired.omegaRadiansPerSecond
                        );
                    }
                    drivetrain.drive(desired);
                    SmartDashboard.putNumber("Movement speed", pitch * Constants.PITCH_MULTIPLIER);
                }
                break;

            case STOP:
                if(Math.abs(pitch) >= Constants.LEVEL_PITCH) {
                    currentState = AutoBalanceStates.FIX_TILT; 
                }
                // drivetrain.drive(new ChassisSpeeds(0, 0, 0));
                drivetrain.setWheelLock();
                SmartDashboard.putNumber("Movement speed", 0.0);
                break;  
                }
            
            return  true;    

            // case PITCH_UP:
            // //if level, stop
            //     // if (pitch < Constants.LEVEL_PITCH){
            //     //     currentState = AutoBalanceStates.PITCH_DOWN;
            //     // }
            //     // else if(Math.abs(pitch) <=Constants.LEVEL_PITCH){
            //     //     currentState = AutoBalanceStates.STOP;
            //     // }

            //     if(Math.abs(pitch) <= Constants.LEVEL_PITCH) {
            //         currentState = AutoBalanceStates.STOP; 
            //     }
            //     else{
            //         drivetrain.drive(new ChassisSpeeds(0.2 * (pitch / 100), 0, 0));
            //     }
            //     break;
                
                
            //     case PITCH_DOWN:
                    
                
            //     // if (pitch > Constants.LEVEL_PITCH){
            //         //     currentState = AutoBalanceStates.PITCH_UP;
            //         // }
            //         if(Math.abs(pitch) <= Constants.LEVEL_PITCH){
            //             currentState = AutoBalanceStates.STOP;
            //         }
            //         else{
            //             // drivetrain.drive(new ChassisSpeeds(-1.5, 0, 0));
            //             drivetrain.drive(new ChassisSpeeds(0.2 * (pitch / 100), 0, 0));
            //         }
            //         break;
            }
            
            
        }
        