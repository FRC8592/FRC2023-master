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
        currentState = AutoBalanceStates.DRIVE_FORWARD;
        double pitch = drivetrain.getPitch();
        switch (currentState){
            
            case DRIVE_FORWARD:
            

                //check if pitch is within 1.5 degrees of 0 to confirm that the robot is in a level state
                if (Math.abs(pitch) <= Constants.LEVEL_PITCH){
                    // System.out.println("DRIVE_FORWARD");
                    drivetrain.drive(new ChassisSpeeds(0.7, 0, 0)); //the slower the better
                }
                //if pitched up
                else if(pitch > Constants.LEVEL_PITCH)
                {
                    currentState = AutoBalanceStates.PITCH_UP;
                }
                //if pitched down
                else{
                    currentState = AutoBalanceStates.PITCH_DOWN;
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
                    // System.out.println("PITCH_UP" + 0.2 * (pitch / 100));
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
                        // System.out.println("PITCH_DOWN");
                        drivetrain.drive(new ChassisSpeeds(-1.5, 0, 0));
                    }
                    break;
                    
                    
                case STOP:
                    // System.out.println("STOP");
                    drivetrain.drive(new ChassisSpeeds(0, 0, 0));
                    break;  
                }
                
                return  true;
            }
            
            
        }
        