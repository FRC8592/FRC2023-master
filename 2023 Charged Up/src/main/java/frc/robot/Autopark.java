package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autopark {

    public enum AutoBalanceStates {
        DRIVE_FORWARD,
        FIX_TILT,
        STOP;
    }
    
    AutoBalanceStates currentState = AutoBalanceStates.DRIVE_FORWARD;
    
    public boolean balance(Drivetrain drivetrain){
        double pitch = drivetrain.getPitch();
        // System.out.println(currentState.toString() + pitch);
        
        switch (currentState){
            
            case DRIVE_FORWARD:
                //if pitched up
                if(Math.abs(pitch) >= Constants.LEVEL_PITCH) {
                    currentState = AutoBalanceStates.FIX_TILT; 
                }
                else {
                    drivetrain.drive(new ChassisSpeeds(0.7, 0, 0)); //the slower the better
                    SmartDashboard.putNumber("Movement speed", 0.7);
                }
                break;
            
            case FIX_TILT:

                if(Math.abs(pitch) <= Constants.LEVEL_PITCH) {
                    currentState = AutoBalanceStates.STOP; 
                }
                else{
                    drivetrain.drive(new ChassisSpeeds(0.2 * (pitch / 100), 0, 0));
                    SmartDashboard.putNumber("Movement speed", pitch * Constants.PITCH_MULTIPLIER);
                }
                break;

            case STOP:
                if(Math.abs(pitch) >= Constants.LEVEL_PITCH) {
                    currentState = AutoBalanceStates.FIX_TILT; 
                }
                drivetrain.drive(new ChassisSpeeds(0, 0, 0));
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
        