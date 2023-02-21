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

public class Lift {
    
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
    private final double MAX_ACCELERATION_UP_LIFT = 1600d;
    private final double MAX_ACCELERATION_DOWN_LIFT = 800d;
    private final double MAX_ACCELERATION_TILT_UP = 3000d;
    private final double MAX_ACCELERATION_TILT_DOWN = 100d;

    // Velocity in RPM
    private final double MAX_VELOCITY_UP_LIFT = 1600d;
    private final double MAX_VELOCITY_DOWN_LIFT = 800d;
    private final double MAX_VELOCITY_TILT_UP = 1200d;
    private final double MAX_VELOCITY_TILT_DOWN = 60d;

    // Current in Amps
    private final int MAX_CURRENT_LIFT = 30; // Amps
    private final int MAX_CURRENT_TILT = 30; // Amps
    
    // Elevator States
    private Heights desiredHeight = Heights.STOWED;
    public enum Heights {
        STOWED(0.0, 0.0),
        STALL(0.0, 0.0),
        PRIME(0.0, Constants.TILT_MAX_ROTATIONS),
        MID(Constants.LIFT_MAX_ROTATIONS / 2, Constants.TILT_MAX_ROTATIONS),
        HIGH(Constants.LIFT_MAX_ROTATIONS, Constants.TILT_MAX_ROTATIONS),
        ;

        double height, tilt;
        Heights(double height, double tilt) {
            this.height = height;
            this.tilt = tilt;
        }

        double getHeight() {
            return height;
        }

        double getTilt() {
            return tilt;
        }
    }

    // Did not set tilt down max accel and velocity
    public Lift() {
        liftMotor = new CANSparkMax(Constants.ELEVATOR_LIFT_MOTOR_ID, MotorType.kBrushless);
        tiltMotor = new CANSparkMax(Constants.ELEVATOR_TILT_MOTOR_ID, MotorType.kBrushless);
        liftCtrl = liftMotor.getPIDController();
        tiltCtrl = tiltMotor.getPIDController();
        liftEncoder = liftMotor.getEncoder();
        tiltEncoder = tiltMotor.getEncoder();

        // PID Constants for lift taken from 1885 2019 Elevator
        liftCtrl.setP(0.0005, PID_UP_SLOT_LIFT);
        liftCtrl.setI(0.0, PID_UP_SLOT_LIFT);
        liftCtrl.setD(0.0, PID_UP_SLOT_LIFT);
        liftCtrl.setFF(0.000391419, PID_UP_SLOT_LIFT);

        liftCtrl.setP(0.0005, PID_DOWN_SLOT_LIFT);
        liftCtrl.setI(0.0, PID_DOWN_SLOT_LIFT);
        liftCtrl.setD(0.0, PID_DOWN_SLOT_LIFT);
        liftCtrl.setFF(0.000391419, PID_DOWN_SLOT_LIFT);

        tiltCtrl.setP(0.0001, PID_TILT_UP_SLOT);
        tiltCtrl.setI(0.0, PID_TILT_UP_SLOT);
        tiltCtrl.setD(-0.00001, PID_TILT_UP_SLOT);

        tiltCtrl.setP(0.00001, PID_TILT_DOWN_SLOT);
        tiltCtrl.setI(0.0, PID_TILT_DOWN_SLOT);
        tiltCtrl.setD(0.0, PID_TILT_DOWN_SLOT);

        liftCtrl.setSmartMotionMaxVelocity(MAX_VELOCITY_DOWN_LIFT, PID_DOWN_SLOT_LIFT);
        liftCtrl.setSmartMotionMaxVelocity(MAX_VELOCITY_UP_LIFT, PID_UP_SLOT_LIFT);
        tiltCtrl.setSmartMotionMaxVelocity(MAX_VELOCITY_TILT_UP, PID_TILT_UP_SLOT);

        liftCtrl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, PID_UP_SLOT_LIFT);
        liftCtrl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, PID_UP_SLOT_LIFT);
        tiltCtrl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, PID_TILT_UP_SLOT);
        liftCtrl.setSmartMotionMaxAccel(MAX_ACCELERATION_DOWN_LIFT, PID_DOWN_SLOT_LIFT);
        liftCtrl.setSmartMotionMaxAccel(MAX_ACCELERATION_UP_LIFT, PID_UP_SLOT_LIFT);
        tiltCtrl.setSmartMotionMaxAccel(MAX_ACCELERATION_TILT_UP, PID_TILT_UP_SLOT);

        liftEncoder.setPosition(0);
        tiltEncoder.setPosition(0);

        liftMotor.setIdleMode(IdleMode.kBrake);
        tiltMotor.setIdleMode(IdleMode.kBrake);

        liftMotor.setSmartCurrentLimit(MAX_CURRENT_LIFT);
        tiltMotor.setSmartCurrentLimit(MAX_CURRENT_TILT);

        // Consider dividing the upper soft limit by the sin of the angle to get the actual upper max
        liftMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.LIFT_MAX_ROTATIONS);
        liftMotor.setSoftLimit(SoftLimitDirection.kForward, 0f);

        tiltMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.TILT_MAX_ROTATIONS);
        tiltMotor.setSoftLimit(SoftLimitDirection.kForward, 5f);

        tiltMotor.setInverted(true);
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

        SmartDashboard.putNumber("Elevator/Lift Rotations", rawLift);
        SmartDashboard.putNumber("Elevator/Tilt Rotations", rawTilt);
        SmartDashboard.putNumber("Elevator/Tilt Current", tiltMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator/Lift Current", liftMotor.getOutputCurrent());
        SmartDashboard.putString("Elevator/Elevator State", desiredHeight.name());
    }

    // Resets encoders and potentially other sensors
    public void reset() {
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

        switch(desiredHeight) {
            case STOWED: // Make sure lift retracts before 4 bar
                liftCtrl.setReference(0.0, ControlType.kSmartMotion);
                if (rawLift >= -5.0) {
                    tiltCtrl.setReference(0.0, ControlType.kSmartMotion, PID_TILT_DOWN_SLOT, getTiltFeedForward(false));
                } else {
                    tiltCtrl.setReference(0.0, ControlType.kSmartVelocity, PID_TILT_DOWN_SLOT, getTiltFeedForward(false));
                }
                break;
            case STALL: // Hold both elevator and 4 bar in place
                liftCtrl.setReference(0.0, ControlType.kSmartVelocity);
                tiltCtrl.setReference(0.0, ControlType.kSmartVelocity, PID_UP_SLOT_LIFT);
                break;
            case PRIME:
                liftCtrl.setReference(0.0, ControlType.kSmartVelocity);
                tiltCtrl.setReference(Constants.TILT_MAX_ROTATIONS, ControlType.kSmartMotion, PID_TILT_UP_SLOT, getTiltFeedForward(true));
                break;
            default: // Mid or high
                tiltCtrl.setReference(Constants.TILT_MAX_ROTATIONS, ControlType.kSmartMotion, PID_TILT_UP_SLOT, getTiltFeedForward(true));
                if (Math.abs(rawTilt - Constants.TILT_MAX_ROTATIONS) <= 3.0 && desiredHeight != Heights.PRIME) {
                    liftCtrl.setReference(desiredHeight.getHeight(), ControlType.kSmartMotion);
                } else {
                    liftCtrl.setReference(0.0, ControlType.kSmartVelocity);
                }
                break;
        }

        // if (desiredHeight == Heights.STOWED) { // Make sure lift retracts before 4 bar
        //     liftCtrl.setReference(0.0, ControlType.kSmartMotion);
        //     if (rawLift >= -5.0) {
        //         tiltCtrl.setReference(0.0, ControlType.kSmartMotion, PID_TILT_DOWN_SLOT, getTiltFeedForward(false));
        //     } else {
        //         tiltCtrl.setReference(0.0, ControlType.kSmartVelocity, PID_TILT_DOWN_SLOT, getTiltFeedForward(false));
        //     }
        // } else { // Make sure 4 bar extends before lift
        //     tiltCtrl.setReference(Constants.TILT_MAX_ROTATIONS, ControlType.kSmartMotion, PID_TILT_UP_SLOT, getTiltFeedForward(true));
        //     if (Math.abs(rawTilt - Constants.TILT_MAX_ROTATIONS) <= 3.0 && desiredHeight != Heights.PRIME) {
        //         liftCtrl.setReference(desiredHeight.getHeight(), ControlType.kSmartMotion);
        //     } else {
        //         liftCtrl.setReference(0.0, ControlType.kSmartVelocity);
        //     }
        // }
    }

    // Sets the desired Height state of the elevator
    public void set(Heights desired) {
        desiredHeight = desired;
    }

    // Gets the amount we scale down the drivetrain speed if we are lifted passed a specific height and/or angle
    public double getDriveReduction() {
        // double rawLift;
        // if (Robot.isReal()) {
        //     rawLift = liftEncoder.getPosition();
        // } else {
        //     rawLift = liftMotor.getAnalog(Mode.kAbsolute).getPosition();
        // }

        // double heightInches = motorRotationsToInches(rawLift);

        // if (heightInches >= Constants.LIFTED_DRIVING_LIMIT_THRESHOLD) {
        //     return 0.25;
        // } else {
        //     return 0;
        // }
        return 0.0;
    }

    private double getTiltFeedForward(boolean up) {
        double curRots = tiltEncoder.getPosition();
        double maxRots = Constants.TILT_MAX_ROTATIONS;
        if (up) {
            return -((maxRots-curRots)/maxRots*12);
        }
        return (curRots/maxRots)*2;
    }
}