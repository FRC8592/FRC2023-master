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
    private final int PID_TILT_SLOT = 0;

    // Acceleration in inches per second per second
    private final double MAX_ACCELERATION_UP_LIFT = 2d;
    private final double MAX_ACCELERATION_DOWN_LIFT = 1d; // Acceleration downwards is considerably slower than acceleration upwards
    private final double MAX_ACCELERATION_TILT = 6d;

    // Velocity in inches per second
    private final double MAX_VELOCITY_UP_LIFT = 1d;
    private final double MAX_VELOCITY_DOWN_LIFT = 1d;
    private final double MAX_VELOCITY_TILT = 1d;

    private final int MAX_CURRENT_LIFT = 10; // Amps
    private final int MAX_CURRENT_TILT = 10; // Amps

    private final double kPulleyDiameterInches = 2.0;
    private final double kMotorRotationsToHeightInches = Constants.LIFT_GEARBOX_RATIO * 2 * Math.PI * kPulleyDiameterInches;
    
    private static final double kIntakeHeight = 0d; // Height to intake from
    private static final double kDropHeightInches = 0; // The height above the top of the node we want to drop from

    private Heights prevHeight = Heights.STOWED;
    private Heights desiredHeight = Heights.STOWED;

    private double kMinimumAngleToLiftThresholdDegrees = 15; // Elevator must be tilted atleast this many degrees to start lifting to non-stowed height
    private double kMaximumHeightToTiltThresholdDegrees = 1; // Elevator must be lifted at max this many inches to tilt back to STOWED angle

    private boolean shouldHold = false;

    public enum Heights {
        STOWED(kDropHeightInches, 0),
        INTAKE(kIntakeHeight + 5.0, 60),
        MID(kDropHeightInches + 35.0, 60),
        HIGH(kDropHeightInches + 46.0, 60);

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

        tiltCtrl.setP(0.0005, PID_TILT_SLOT);
        tiltCtrl.setI(0.0, PID_TILT_SLOT);
        tiltCtrl.setD(0.0, PID_TILT_SLOT);
        tiltCtrl.setFF(0.0, PID_TILT_SLOT);

        // Figure out how to convert inches per second to RPM and inches per second per second to RPM per second
        liftCtrl.setSmartMotionMaxVelocity(MAX_VELOCITY_DOWN_LIFT, PID_DOWN_SLOT_LIFT);
        liftCtrl.setSmartMotionMaxVelocity(MAX_VELOCITY_UP_LIFT, PID_UP_SLOT_LIFT);
        tiltCtrl.setSmartMotionMaxVelocity(MAX_VELOCITY_TILT, PID_TILT_SLOT);

        liftCtrl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, PID_UP_SLOT_LIFT);
        liftCtrl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, PID_UP_SLOT_LIFT);
        tiltCtrl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, PID_TILT_SLOT);
        liftCtrl.setSmartMotionMaxAccel(MAX_ACCELERATION_DOWN_LIFT, PID_DOWN_SLOT_LIFT);
        liftCtrl.setSmartMotionMaxAccel(MAX_ACCELERATION_UP_LIFT, PID_UP_SLOT_LIFT);
        liftCtrl.setSmartMotionMaxAccel(MAX_ACCELERATION_TILT, PID_TILT_SLOT);

        // The actual stowed height is stored as the 'zero' for the lift
        liftEncoder.setPosition((float)inchesToMotorRotations(Heights.STOWED.getHeight()));
        tiltEncoder.setPosition(0);

        liftMotor.setIdleMode(IdleMode.kBrake);
        tiltMotor.setIdleMode(IdleMode.kBrake);

        liftMotor.setSmartCurrentLimit(MAX_CURRENT_LIFT);
        tiltMotor.setSmartCurrentLimit(MAX_CURRENT_TILT);

        // Consider dividing the upper soft limit by the sin of the angle to get the actual upper max
        liftMotor.setSoftLimit(SoftLimitDirection.kForward, (float)inchesToMotorRotations(Heights.HIGH.getHeight()));
        liftMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)inchesToMotorRotations(Heights.STOWED.getHeight()));

        tiltMotor.setSoftLimit(SoftLimitDirection.kForward, (float)degreesToMotorRotations(Heights.HIGH.getTilt()));
        tiltMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)degreesToMotorRotations(Heights.STOWED.getTilt()));
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

        // SmartDashboard.putNumber("Elevator/Linear Height Inches", getLinearHeightInches());
        SmartDashboard.putNumber("Elevator/Linear Height Inches", motorRotationsToInches(rawLift));
        SmartDashboard.putString("Elevator/Desired State", prevHeight.name());
        SmartDashboard.putNumber("Elevator/Absolute Height Inches", getAbsoluteHeightInches(rawLift, rawTilt));
        SmartDashboard.putNumber("Elevator/Tilt Degrees", motorRotationsToDegrees(rawTilt));
    }

    // Resets encoders and potentially other sensors
    public void reset() {
        liftEncoder.setPosition((float)inchesToMotorRotations(Heights.STOWED.getHeight()));
        tiltEncoder.setPosition(0);
    }

    // Called periodically to update outputs for the elevator given inputs
    public void periodic() {
        double rawLift, rawTilt;
        if (Robot.isReal()) {
            rawLift = liftEncoder.getPosition();
            rawTilt = tiltEncoder.getPosition();
        } else {
            rawLift = liftMotor.getAnalog(Mode.kAbsolute).getPosition();
            rawTilt = tiltMotor.getAnalog(Mode.kAbsolute).getPosition();
        }

        double tiltDegrees = motorRotationsToDegrees(rawTilt);
        double liftInches = motorRotationsToInches(rawLift);
        double desiredInches = desiredHeight.getHeight();

        if (shouldHold) { // Nothing is being pressed, so the elevator should not move
            liftCtrl.setReference(0, ControlType.kVelocity);
            tiltCtrl.setReference(0, ControlType.kVelocity);
        } else {
            if (desiredHeight == Heights.STOWED) { // If we want to go back into the resting state
                if (liftInches <= kMaximumHeightToTiltThresholdDegrees) { // If the lift isn't currently extended
                    tiltCtrl.setReference(degreesToMotorRotations(desiredHeight.getTilt()), ControlType.kSmartMotion, PID_TILT_SLOT);
                } else {
                    tiltCtrl.setReference(0, ControlType.kSmartVelocity, PID_TILT_SLOT);
                }
            } else {
                if (desiredHeight.ordinal() >= this.prevHeight.ordinal()) { // If we want to lift UP
                    if (tiltDegrees >= kMinimumAngleToLiftThresholdDegrees) { // If the elevator is tilted enough
                        liftCtrl.setReference(inchesToMotorRotations(desiredInches), ControlType.kSmartMotion, PID_UP_SLOT_LIFT);
                    } else {
                        liftCtrl.setReference(0, ControlType.kSmartVelocity, PID_UP_SLOT_LIFT);
                    }
                } else { // If we want to lift DOWN
                    if (tiltDegrees >= kMinimumAngleToLiftThresholdDegrees) { // If the elevator is tilted enough
                        liftCtrl.setReference(inchesToMotorRotations(desiredInches), ControlType.kSmartMotion, PID_DOWN_SLOT_LIFT);
                    } else {
                        liftCtrl.setReference(0, ControlType.kSmartVelocity, PID_DOWN_SLOT_LIFT);
                    }
                }
            }

            prevHeight = desiredHeight;
        }
    }

    public void hold(boolean shouldHold) {
        this.shouldHold = shouldHold;
    }

    // Spin Lift motor 5% of [Left Joystick Y Axis] value
    public void testPlan1Lift(double pct) {
        liftMotor.set(pct/20);
    }

    // Spin Tilt motor 5% of [Left Joystick Y Axis] value
    public void testPlan1Tilt(double pct) {
        tiltMotor.set(pct/20);
    }

    // Goes to a height (5 inches) upon [Right Bumper] press
    public void testPlan2Lift() {
        liftCtrl.setReference(inchesToMotorRotations(5d), ControlType.kSmartMotion);
    }

    // Go to desired angle (45 degrees) upon [Right Bumper] press
    public void testPlan2Tilt() {
        tiltCtrl.setReference(degreesToMotorRotations(45), ControlType.kSmartMotion);
    }

    // Go to the desired angle (up, in)
    public void testPlan3Tilt(Heights height) {
        this.prevHeight = height;
        tiltCtrl.setReference(height.getTilt(), ControlType.kSmartMotion);
    }

    // Go to different heights (stowed, mid, high)
    public void testPlan3Lift(Heights height) {
        // If the desired height is higher than the previous height go to the up slot, otherwise go to down slot for PID
        if (height.ordinal() >= this.prevHeight.ordinal()) { 
            liftCtrl.setReference(height.getHeight(), ControlType.kSmartMotion, PID_UP_SLOT_LIFT);
        } else {
            liftCtrl.setReference(height.getHeight(), ControlType.kSmartMotion, PID_DOWN_SLOT_LIFT);
        }

        this.prevHeight = height;
    }

    // Sets the desired Height state of the elevator
    public void setHeight(Heights desired) {
        desiredHeight = desired;
        shouldHold = false;
    }

    // Gets the height of the elevator accounting for the tilt
    private double getAbsoluteHeightInches(double rawLift, double rawTilt) {
        return motorRotationsToInches(rawLift) * Math.sin(motorRotationsToDegrees(rawTilt));
    }

    // Converts motor rotations to degrees
    private double motorRotationsToDegrees(double motorRotations) {
        return Constants.TILT_GEARBOX_RATIO * motorRotations;
    }

    // Converts motor rotations to inches
    private double motorRotationsToInches(double rotations) {
        return rotations*kMotorRotationsToHeightInches;
    }

    // Converts inches to motor rotations
    private double inchesToMotorRotations(double inches) {
        return inches/kMotorRotationsToHeightInches;
    }

    // Converts degrees to motor rotations
    private double degreesToMotorRotations(double degrees) {
        return degrees / Constants.TILT_GEARBOX_RATIO;
    }

    // Gets the amount we scale down the drivetrain speed if we are lifted passed a specific height and/or angle
    public double getDriveReduction() {
        double rawLift;
        if (Robot.isReal()) {
            rawLift = liftEncoder.getPosition();
        } else {
            rawLift = liftMotor.getAnalog(Mode.kAbsolute).getPosition();
        }

        double heightInches = motorRotationsToInches(rawLift);

        if (heightInches >= Constants.LIFTED_DRIVING_LIMIT_THRESHOLD) {
            return 0.25;
        } else {
            return 0;
        }
    }
}