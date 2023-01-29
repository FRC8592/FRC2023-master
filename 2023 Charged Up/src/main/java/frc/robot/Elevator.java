package frc.robot;

import java.lang.reflect.Method;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Elevator {
    private CANSparkMax elevatorLiftMotor;
    private CANSparkMax elevatorTiltMotor;
    private RelativeEncoder liftEncoder;
    private RelativeEncoder tiltEncoder;

    private SparkMaxPIDController liftCtrl;
    private SparkMaxPIDController tiltCtrl;

    private Piece currentPiece;

    private final int kPulleyDiameterInches = 2;
    private final double kMotorRotationsToHeightInches = Constants.LIFT_GEARBOX_RATIO * 2 * Math.PI * kPulleyDiameterInches;

    public enum ScoreHeight {
        STOWED(Constants.STOWED_HEIGHT_CONE, Constants.STOWED_HEIGHT_CUBE),
        LOW(Constants.LOW_HEIGHT_CONE, Constants.LOW_HEIGHT_CUBE),
        MID(Constants.MID_HEIGHT_CONE, Constants.MID_HEIGHT_CUBE),
        HIGH(Constants.HIGH_HEIGHT_CONE, Constants.HIGH_HEIGHT_CUBE);

        double coneHeight, cubeHeight;
        ScoreHeight(double cone, double cube) {
            this.coneHeight = cone;
            this.cubeHeight = cube;
        }

        public double getConeHeight() {
            return coneHeight;
        }

        public double getCubeHeight() {
            return cubeHeight;
        }
    }

    public enum Piece {
        CONE,
        CUBE
    }

    // For all:
    //  Put in brake mode
    //  Hard cap drivetrain accel when extended past mid (extension point subject to change)
    //      these two WIP in robot.java
    // Bring claw to desired position
    public Elevator() {
        elevatorLiftMotor = new CANSparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        elevatorTiltMotor = new CANSparkMax(Constants.ANGLE_MOTOR_ID, MotorType.kBrushless);
        
        liftEncoder = elevatorLiftMotor.getEncoder();
        tiltEncoder = elevatorTiltMotor.getEncoder();

        liftCtrl = elevatorLiftMotor.getPIDController();
        liftCtrl.setP(0.001);
        liftCtrl.setI(0d);
        liftCtrl.setD(0d);

        tiltCtrl = elevatorTiltMotor.getPIDController();
        tiltCtrl.setP(0.001);
        tiltCtrl.setI(0d);
        tiltCtrl.setD(0d);

        elevatorLiftMotor.setIdleMode(IdleMode.kBrake);
        elevatorTiltMotor.setIdleMode(IdleMode.kBrake);

        liftEncoder.setPosition(0);
        tiltEncoder.setPosition(Constants.ARM_ANGLE_IN / 360.0);
    }

    public ScoreHeight upOneLevel (ScoreHeight current) {
        switch(current) {
            case LOW:
                return ScoreHeight.MID;
            case MID:
                return ScoreHeight.HIGH;
            default:
                return current;
        }
    }

    public ScoreHeight downOneLevel (ScoreHeight current) {
        switch(current) {
            case MID:
                return ScoreHeight.LOW;
            case HIGH:
                return ScoreHeight.MID;
            default:
                return current;
        }
    }

    public void setPiece(Piece piece) {
        currentPiece = piece;
    }

    public void liftTo(ScoreHeight height) {
        if (height == ScoreHeight.STOWED) {
            // liftCtrl.setReference(Constants.ARM_ANGLE_IN, ControlType.kPosition);
            tilt(false);
        } else {
            // liftCtrl.setReference(Constants.ARM_ANGLE_OUT, ControlType.kPosition);
            tilt(true);
        }

        double desiredHeight = currentPiece == Piece.CUBE ? height.getCubeHeight() : height.getConeHeight();
        liftCtrl.setReference(desiredHeight, ControlType.kPosition);
    }

    public void tilt(boolean out) {
        if (out) {
            tiltCtrl.setReference(Constants.ARM_ANGLE_OUT / 360d, ControlType.kPosition);
        } else {
            tiltCtrl.setReference(Constants.ARM_ANGLE_IN / 360d, ControlType.kPosition);
        }
    }

    // Call on init, will reset encoders and make sure elevator is in frame perimeter
    public void reset() {
        liftTo(ScoreHeight.STOWED);
        liftEncoder.setPosition(0.0);
        tiltEncoder.setPosition(Constants.ARM_ANGLE_IN / 360d); // Convert degrees to rotations
    }

    // Find a way to reduce the drivetrian speed based on elevator height
    public double getDriveReduction() {
        return getAbsoluteHeight();
    }

    private double getAbsoluteHeight() {
        return getLiftHeight() * Math.sin(getTiltAngle());
    }

    private double getLiftHeight() {
        return liftEncoder.getPosition() * kMotorRotationsToHeightInches;
    }

    private double getTiltAngle() {
        return tiltEncoder.getPosition() * 360;
    }

}