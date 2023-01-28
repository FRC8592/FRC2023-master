package frc;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.Drivetrain;

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
        switch (currentState){
            case DRIVE_FORWARD:
                if (Math.abs(pitch)<= Constants.LEVEL_PITCH){
                    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.5, 0, 0,  drivetrain.getGyroscopeRotation()));
                }
                else if(pitch > Constants.LEVEL_PITCH)
                {
                    currentState = AutoBalanceStates.PITCH_UP;
                }
                else{
                    currentState = AutoBalanceStates.PITCH_DOWN;
                }

            case PITCH_UP:
                if(Math.abs(pitch)<=Constants.LEVEL_PITCH){
                    currentState = AutoBalanceStates.STOP;

                }
                else if (pitch < Constants.LEVEL_PITCH){
                    currentState = AutoBalanceStates.PITCH_DOWN;
                }
                else{
                    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.5, 0, 0,  drivetrain.getGyroscopeRotation()));
                }

            case PITCH_DOWN:
                if(Math.abs(pitch)<=Constants.LEVEL_PITCH){
                    currentState = AutoBalanceStates.STOP;

            }
                else if (pitch > Constants.LEVEL_PITCH){
                    currentState = AutoBalanceStates.PITCH_UP;
            }
                else{
                    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-0.5, 0, 0,  drivetrain.getGyroscopeRotation()));
            }
            
            case STOP:
                drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0,  drivetrain.getGyroscopeRotation()));
        }

        return  true;
    }
}
