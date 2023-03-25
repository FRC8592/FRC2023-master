package frc.robot;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Autopark {

    public enum AutoBalanceStates {
        DRIVE_FORWARD,
        FIX_TILT,
        STOP;
    }
    
    AutoBalanceStates currentState;
    Timer timer;
    private PIDController balancePID;
    private double initialDist = 0, finalDist = 0, speed;
    private boolean firstFrame = true;

    public Autopark(){
        currentState = AutoBalanceStates.DRIVE_FORWARD;
        timer = new Timer();
        balancePID = new PIDController(Constants.AUTOBALANCE_kP, Constants.AUTOBALANCE_kI, Constants.AUTOBALANCE_kD);
        balancePID.setTolerance(Constants.LEVEL_PITCH);
    }

    public boolean balanceTest1(Drivetrain drivetrain) {
        double pitch = drivetrain.getRoll();
        boolean notBalanced = true;
        System.out.println(currentState.toString() + " " + pitch);

        if (Math.abs(pitch) >= Constants.LEVEL_PITCH) {
            speed = -Math.signum(pitch) * Math.min(Math.abs(pitch * Constants.PITCH_MULTIPLIER), Constants.AUTOBALANCE_MAX_SPEED);
            drivetrain.drive(new ChassisSpeeds(speed, 0, 0));
        } else {
            drivetrain.setWheelLock();
            speed = 0;
        }
        SmartDashboard.putNumber("Movement speed", 0);

        return notBalanced;
    }

    public boolean balanceTest2(Drivetrain drivetrain) {
        double pitch = drivetrain.getRoll();
        boolean notBalanced = true;
        System.out.println(currentState.toString() + " " + pitch);

        if (Math.abs(pitch) >= Constants.LEVEL_PITCH) {
            speed = -Math.signum(pitch) * Math.min(Math.abs(balancePID.calculate(pitch, 0)), Constants.AUTOBALANCE_MAX_SPEED);
            drivetrain.drive(new ChassisSpeeds(speed, 0, 0));
        } else {
            drivetrain.setWheelLock();
            speed = 0;
        }
        SmartDashboard.putNumber("Movement speed", speed);

        return notBalanced;
    }

    public boolean balanceTest3(Drivetrain drivetrain) {
        double pitch = drivetrain.getRoll();
        boolean notBalanced = true;
        System.out.println(currentState.toString() + " " + pitch);

        if (Math.abs(pitch) >= Constants.LEVEL_PITCH) {
            speed = -Math.signum(pitch) * Math.min(Math.abs(balancePID.calculate(pitch, 0)), Constants.AUTOBALANCE_MAX_SPEED);
            drivetrain.drive(new ChassisSpeeds(balancePID.calculate(pitch, 0), 0, 0));
        } else {
            drivetrain.setWheelLock();
            speed = 0;
        }
        SmartDashboard.putNumber("Movement speed", speed);

        return notBalanced;
    }
    
    public boolean balance(Drivetrain drivetrain){
        double pitch = drivetrain.getRoll();
        boolean notBalanced = true;
        // System.out.println(currentState.toString() + " " + pitch);

        if (Math.abs(pitch) >= Constants.LEVEL_PITCH && Math.abs(finalDist - initialDist) <= 0.75) {
            if (firstFrame) {
                firstFrame = false;
                initialDist = drivetrain.getCurrentPos().getX();
            }
            finalDist = drivetrain.getCurrentPos().getX();
            drivetrain.drive(new ChassisSpeeds(balancePID.calculate(pitch, 0), 0, 0));
            SmartDashboard.putNumber("Movement speed", balancePID.calculate(pitch, 0));
        } else {
            initialDist = 0;
            finalDist = 0;
            drivetrain.setWheelLock();
            SmartDashboard.putNumber("Movement speed", 0);
        }
        
        // switch (currentState){
            
        //     case DRIVE_FORWARD:
        //         //if pitched up
        //         timer.start();
        //         if(Math.abs(pitch) >= Constants.LEVEL_PITCH && timer.get() >= 2.0) {
        //             currentState = AutoBalanceStates.FIX_TILT; 
        //         }
        //         else {
        //             drivetrain.drive(new ChassisSpeeds(-0.7, 0, 0)); //the slower the better
        //             SmartDashboard.putNumber("Movement speed", 0.7);
        //         }
        //         break;
            
        //     case FIX_TILT:
                
        //         if(Math.abs(pitch) <= Constants.LEVEL_PITCH) {
        //             currentState = AutoBalanceStates.STOP; 
        //         }
        //         else{
        //             // drivetrain.drive(new ChassisSpeeds(-(pitch * Constants.PITCH_MULTIPLIER), 0, 0));
        //             drivetrain.drive(new ChassisSpeeds(balancePID.calculate(pitch, 0), 0, 0));

        //             // SmartDashboard.putNumber("Movement speed", pitch * Constants.PITCH_MULTIPLIER);
        //             SmartDashboard.putNumber("Movement speed", balancePID.calculate(pitch, 0));

        //         }
        //         break;

        //     case STOP:
        //         if(Math.abs(pitch) >= Constants.LEVEL_PITCH) {
        //             currentState = AutoBalanceStates.FIX_TILT; 
        //         }
        //         // drivetrain.drive(new ChassisSpeeds(0, 0, 0));
        //         drivetrain.setWheelLock();
        //         SmartDashboard.putNumber("Movement speed", 0.0);
        //         break;  
        //         }
            
                return notBalanced;
            }
            
            
        }
        