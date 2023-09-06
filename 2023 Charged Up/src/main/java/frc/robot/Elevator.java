package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {
    
    // Elevator Lift Hardware
    private CANSparkMax liftMotor;
    private SparkMaxPIDController liftCtrl;
    private RelativeEncoder liftEncoder;
    
    // Elevator Tilt Hardware
    private CANSparkMax tiltMotor;
    private SparkMaxPIDController tiltCtrl;
    private RelativeEncoder tiltEncoder;

    // Different PID constants for moving upwards and downwards
    private final int PID_UP_SLOT_LIFT = 0;
    private final int PID_DOWN_SLOT_LIFT = 1;
    private final int PID_TILT_UP_SLOT = 0;
    private final int PID_TILT_DOWN_SLOT = 1;

    // Acceleration in RPM per second
    private final double MAX_ACCELERATION_UP_LIFT = 6000d;
    private final double MAX_ACCELERATION_DOWN_LIFT = 6000d;
    private final double MAX_ACCELERATION_TILT_UP = 10000d;
    private final double MAX_ACCELERATION_TILT_DOWN = 5000d;

    // Velocity in RPM
    private final double MAX_VELOCITY_UP_LIFT = 6000d;
    private final double MAX_VELOCITY_DOWN_LIFT = 6000d;
    private final double MAX_VELOCITY_TILT_UP = 4000d;
    private final double MAX_VELOCITY_TILT_DOWN = 6000d;

    // Current in Amps
    private final int MAX_CURRENT_LIFT = 30; // Amps
    private final int MAX_CURRENT_TILT = 40; // Amps
    
    // Elevator States
    private Heights desiredHeight = Heights.STOWED;
    public enum Heights {
        STOWED(0.0),
        STALL(0.0),
        PRIME(0.0),
        MID(Constants.LIFT_MAX_ROTATIONS / 2 + 10.0),
        HIGH(Constants.LIFT_MAX_ROTATIONS + 5.0),
        MANUAL(0.0),
        ;

        private double height;
        Heights(double height) {
            this.height = height;
        }

        double getHeight() {
            return height;
        }
    }

    // Did not set tilt down max accel and velocity
    public Elevator() {
        liftMotor = new CANSparkMax(Constants.ELEVATOR_LIFT_MOTOR_ID, MotorType.kBrushless);
        tiltMotor = new CANSparkMax(Constants.ELEVATOR_TILT_MOTOR_ID, MotorType.kBrushless);
        liftCtrl = liftMotor.getPIDController();
        tiltCtrl = tiltMotor.getPIDController();
        liftEncoder = liftMotor.getEncoder();
        tiltEncoder = tiltMotor.getEncoder();

        // PID Constants for lift taken from 1885 2019 Elevator
        liftCtrl.setP(0.00030, PID_UP_SLOT_LIFT);
        liftCtrl.setI(0.0, PID_UP_SLOT_LIFT);
        liftCtrl.setD(0.0, PID_UP_SLOT_LIFT);
        liftCtrl.setFF(0.000391419, PID_UP_SLOT_LIFT);

        liftCtrl.setP(0.00030, PID_DOWN_SLOT_LIFT);
        liftCtrl.setI(0.0, PID_DOWN_SLOT_LIFT);
        liftCtrl.setD(0.0, PID_DOWN_SLOT_LIFT);
        liftCtrl.setFF(0.000391419, PID_DOWN_SLOT_LIFT);

        tiltCtrl.setP(0.000450, PID_TILT_UP_SLOT); //0.000500
        tiltCtrl.setI(0.0, PID_TILT_UP_SLOT);
        tiltCtrl.setD(0.000425, PID_TILT_UP_SLOT); // 0.000375(current as of 9/6/23) .0004
        // tiltCtrl.setFF(0.000075, PID_TILT_UP_SLOT);

        tiltCtrl.setP(0.0001, PID_TILT_DOWN_SLOT);
        tiltCtrl.setI(0.0, PID_TILT_DOWN_SLOT);
        tiltCtrl.setD(0.0, PID_TILT_DOWN_SLOT);

        liftCtrl.setSmartMotionMaxVelocity(MAX_VELOCITY_DOWN_LIFT, PID_DOWN_SLOT_LIFT);
        liftCtrl.setSmartMotionMaxVelocity(MAX_VELOCITY_UP_LIFT, PID_UP_SLOT_LIFT);
        tiltCtrl.setSmartMotionMaxVelocity(MAX_VELOCITY_TILT_UP, PID_TILT_UP_SLOT);
        tiltCtrl.setSmartMotionMaxVelocity(MAX_VELOCITY_TILT_DOWN, PID_TILT_DOWN_SLOT);

        liftCtrl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, PID_UP_SLOT_LIFT);
        liftCtrl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, PID_UP_SLOT_LIFT);
        tiltCtrl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, PID_TILT_UP_SLOT);
        liftCtrl.setSmartMotionMaxAccel(MAX_ACCELERATION_DOWN_LIFT, PID_DOWN_SLOT_LIFT);
        liftCtrl.setSmartMotionMaxAccel(MAX_ACCELERATION_UP_LIFT, PID_UP_SLOT_LIFT);
        tiltCtrl.setSmartMotionMaxAccel(MAX_ACCELERATION_TILT_UP, PID_TILT_UP_SLOT);
        tiltCtrl.setSmartMotionMaxAccel(MAX_ACCELERATION_TILT_DOWN, PID_TILT_DOWN_SLOT);

        liftEncoder.setPosition(0);
        tiltEncoder.setPosition(0);

        liftMotor.setIdleMode(IdleMode.kBrake);
        tiltMotor.setIdleMode(IdleMode.kBrake);

        liftMotor.setSmartCurrentLimit(MAX_CURRENT_LIFT);
        tiltMotor.setSmartCurrentLimit(MAX_CURRENT_TILT);

        liftMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.LIFT_MAX_ROTATIONS);
        liftMotor.setSoftLimit(SoftLimitDirection.kForward, 0f);

        tiltMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.TILT_MAX_ROTATIONS);
        tiltMotor.setSoftLimit(SoftLimitDirection.kForward, 0f);

        tiltMotor.setInverted(true);

        liftCtrl.setSmartMotionAllowedClosedLoopError(0.1, PID_UP_SLOT_LIFT);
        liftCtrl.setSmartMotionAllowedClosedLoopError(0.1, PID_DOWN_SLOT_LIFT);

        // SmartDashboard.putNumber("Desired Max Tilt", Constants.TILT_MAX_ROTATIONS);
    }

    // Logs values to SmartDashboard
    public void writeToSmartDashboard() {
        double rawLift, rawTilt;
        if (Robot.isReal()) {
            rawLift = liftEncoder.getPosition();
            rawTilt = tiltEncoder.getPosition();
        } else {
            rawLift = liftMotor.getAnalog(Mode.kAbsolute).getPosition();
            rawTilt = tiltMotor.getAnalog(Mode.kAbsolute).getPosition();
        }

        double errorTilt;
        switch (desiredHeight) {
            case MANUAL:
            // Fall through to STALL
            case STALL:
                errorTilt = 0;
                break;
            case STOWED:
                errorTilt = -rawTilt;
                break;
            default:
                errorTilt = Constants.TILT_MAX_ROTATIONS - rawTilt;
        }

        // SmartDashboard.putNumber("Elevator Rotations", rawLift);
        // SmartDashboard.putNumber("Elevator Current", liftMotor.getOutputCurrent());
        // SmartDashboard.putString("Elevator State", desiredHeight.name());
        // SmartDashboard.putNumber("Elevator Error Rotations", desiredHeight.getHeight() - rawLift);
        // SmartDashboard.putNumber("Elevator Velocity", liftEncoder.getVelocity());
        
        
        // SmartDashboard.putNumber("Tilt Rotations", rawTilt);
        // SmartDashboard.putNumber("Tilt Current", tiltMotor.getOutputCurrent());
        // SmartDashboard.putNumber("Tilt Error Rotations", errorTilt);
        
    }
 

    // Resets encoders and potentially other sensors to desired start angle
    public void reset(double lift, double tilt) {
        liftEncoder.setPosition(0);
        tiltEncoder.setPosition(0);
    }

    public void update() {
        double rawLift, rawTilt;
        if (Robot.isReal()) {
            rawLift = liftEncoder.getPosition();
            rawTilt = -tiltEncoder.getPosition();
        } else {
            rawLift = liftMotor.getAnalog(Mode.kAbsolute).getPosition();
            rawTilt = tiltMotor.getAnalog(Mode.kAbsolute).getPosition();
        }

        double maxTilt = SmartDashboard.getNumber("Desired Max Tilt", Constants.TILT_MAX_ROTATIONS);

        switch(desiredHeight) {
            case STOWED:
                liftCtrl.setReference(0.0, ControlType.kSmartMotion, PID_DOWN_SLOT_LIFT);
                if (rawLift >= Constants.LIFT_THRESHOLD_TO_STOW) { // Lifted back enough to retract pivot
                    // tiltCtrl.setReference(0.0, ControlType.kSmartMotion, PID_TILT_DOWN_SLOT, getTiltFeedForward(false));
                    tiltCtrl.setReference(0.0, ControlType.kSmartMotion, PID_TILT_DOWN_SLOT);
                } else {
                    tiltMotor.set(0.0);
                }
                break;
            case STALL: // Hold both elevator and 4 bar in place
                liftMotor.set(0.0);
                tiltMotor.set(0.0);
                break;
            case PRIME: // Prepare 4 bar without lifting elevator
                // liftMotor.set(0.0);
                liftCtrl.setReference(0.0, ControlType.kSmartMotion, PID_DOWN_SLOT_LIFT);
                tiltCtrl.setReference(maxTilt, ControlType.kSmartMotion, PID_TILT_UP_SLOT);
                // tiltCtrl.setReference(maxTilt, ControlType.kSmartMotion, PID_TILT_UP_SLOT, getTiltFeedForward(true));
                // tiltCtrl.setReference(maxTilt, ControlType.kSmartMotion, PID_TILT_UP_SLOT, getTiltFeedForward(true));
                break;
            case MANUAL:
                // tiltCtrl.setReference(rawTilt, ControlType.kSmartMotion, PID_TILT_UP_SLOT, getTiltFeedForward(true));
                // liftCtrl.setReference(rawLift, ControlType.kSmartMotion, PID_UP_SLOT_LIFT);
                break;
            default: // Mid or high
                // tiltCtrl.setReference(maxTilt, ControlType.kSmartMotion, PID_TILT_UP_SLOT, getTiltFeedForward(true));
                tiltCtrl.setReference(maxTilt, ControlType.kSmartMotion, PID_TILT_UP_SLOT);
                if (Math.abs(rawTilt) >= Math.abs(Constants.TILT_THRESHOLD_TO_LIFT)) {
                    liftCtrl.setReference(desiredHeight.getHeight(), ControlType.kSmartMotion, PID_UP_SLOT_LIFT);
                } else {
                    liftMotor.set(0.0);
                }
                break;
        }
    }

    // Sets the desired Height state of the elevator
    public void set(Heights desired) {
        desiredHeight = desired;
    }

    public void manualControl(double lift, double tilt) {
        set(Heights.MANUAL);
        liftMotor.set(lift / 2);
        tiltMotor.set(tilt / 2);
        // liftCtrl.setReference(lift * , ControlType.kSmartVelocity, PID_UP_SLOT_LIFT);
        // tiltCtrl.setReference(tilt * , ControlType.kSmartVelocity, PID_UP_SLOT_LIFT);
    }

    public void overrideStow() {
        tiltCtrl.setReference(0.0, ControlType.kSmartMotion, PID_TILT_DOWN_SLOT, 12.0);
    }

    // Gets the amount we scale down the drivetrain speed if we are lifted passed a specific height and/or angle
    public double getDriveReduction() {
        double rawLift;
        if (Robot.isReal()) {
            rawLift = liftEncoder.getPosition(); 
        } else {
            rawLift = liftMotor.getAnalog(Mode.kAbsolute).getPosition();
        }
        return 1.0 - rawLift / Constants.LIFT_MAX_ROTATIONS;
    }

    private double getTiltFeedForward(boolean up) {
        double curRots = tiltEncoder.getPosition();
        double maxRots = Constants.TILT_MAX_ROTATIONS;
        if (up) {
            if (curRots >= Constants.TILT_MAX_ROTATIONS * 0.5 && curRots <= 1.0) {
                return -12.0;
            }
            return 0.0;
        }
        return (curRots/maxRots)*2.5;
    }

    public boolean atTiltReference() {
        if (desiredHeight == Heights.STOWED) {
            return Math.abs(tiltEncoder.getPosition()) <= 2.0;
        } else {
            return Math.abs(tiltEncoder.getPosition() - Constants.TILT_MAX_ROTATIONS) <= 2.0;
        }
    }

    public boolean atReference() {
        boolean atTilt = false;
        if (desiredHeight == Heights.STOWED) {
            atTilt = Math.abs(tiltEncoder.getPosition()) <= 2.0;
        } else {
            atTilt = Math.abs(tiltEncoder.getPosition() - Constants.TILT_MAX_ROTATIONS) <= 2.0;
        }
        return Math.abs(liftEncoder.getPosition() - desiredHeight.getHeight()) <= 2.0 && atTilt;
                
    }
}