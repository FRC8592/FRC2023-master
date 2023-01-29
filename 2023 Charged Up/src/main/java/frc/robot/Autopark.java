package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autopark {

    public enum AutoBalanceStates {
        DRIVE_FORWARD, 
        PITCH_UP,
        PITCH_DOWN,
        STOP;
    }
    
    AutoBalanceStates currentState = AutoBalanceStates.DRIVE_FORWARD;
    
    public boolean balance(Drivetrain drivetrain){
        double pitch = drivetrain.getPitch();
        System.out.println(currentState.toString() + pitch);
        switch (currentState){
            
            case DRIVE_FORWARD:
            

            
            //if pitched up
                if(pitch > Constants.LEVEL_PITCH)
                {
                    currentState = AutoBalanceStates.PITCH_UP;
                }
                //if pitched down
                else if (pitch < -Constants.LEVEL_PITCH){
                    currentState = AutoBalanceStates.PITCH_DOWN;
                }else {
                    drivetrain.drive(new ChassisSpeeds(0.7, 0, 0)); //the slower the better

                }
                break;
            
            case PITCH_UP:
            //if level, stop
                if (pitch < Constants.LEVEL_PITCH){
                    currentState = AutoBalanceStates.PITCH_DOWN;
                }
                else if(Math.abs(pitch) <=Constants.LEVEL_PITCH){
                    currentState = AutoBalanceStates.STOP;
                }
                else{
                    drivetrain.drive(new ChassisSpeeds(0.2 * (pitch / 100), 0, 0));
                }
                break;
                
                
                case PITCH_DOWN:
                    
                
                // if (pitch > Constants.LEVEL_PITCH){
                    //     currentState = AutoBalanceStates.PITCH_UP;
                    // }
                    if(Math.abs(pitch) <= Constants.LEVEL_PITCH){
                        currentState = AutoBalanceStates.STOP;
                    }
                    else{
                        drivetrain.drive(new ChassisSpeeds(-1.5, 0, 0));
                    }
                    break;
                    
                    
                case STOP:
                    drivetrain.drive(new ChassisSpeeds(0, 0, 0));
                    break;  
                }
                
                return  true;
            }
            
            
        }
        