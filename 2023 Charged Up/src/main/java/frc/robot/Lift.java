package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lift {
    
    private CANSparkMax liftMotor;
    private SparkMaxPIDController liftCtrl;
    private RelativeEncoder liftEncoder;

    // Different PID constants for moving upwards and downwards
    private final int PID_UP_SLOT = 0;
    private final int PID_DOWN_SLOT = 1;

    // Acceleration in inches per second per second
    private final double MAX_ACCELERATION_UP = 12d;
    private final double MAX_ACCELERATION_DOWN = 6d; // Acceleration downwards is considerably slower than acceleration upwards

    // Velocity in inches per second
    private final double MAX_VELOCITY_UP = 1d;
    private final double MAX_VELOCITY_DOWN = 1d;

    private final int MAX_CURRENT = 30; // Amps

    // private final float LOWER_SOFT_LIMIT = 0;
    // private final float UPPER_SOFT_LIMIT = 3;

    private final double kPulleyDiameterInches = 2.0; 
    private final double kMotorRotationsToHeightInches = Constants.LIFT_GEARBOX_RATIO * 2 * Math.PI * kPulleyDiameterInches;


    private static final double kConeHangInches = 3; // The distance the cone hangs from the bottom of the intake
    private static final double kDropHeightInches = 6; // The height above the top of the node we want to drop from

    private Heights prevHeight = Heights.STOWED;
    private Pieces piece = Pieces.CONE;

    public enum Heights {
        STOWED(kConeHangInches + kDropHeightInches, false),
        INTAKE(kConeHangInches + 5, false),
        MID(kConeHangInches + kDropHeightInches + 35, true),
        HIGH(kConeHangInches + kDropHeightInches + 46, true);

        double height;
        boolean coneChange;
        Heights(double height, boolean coneChange) {
            this.height = height;
            this.coneChange = coneChange;
        }

        double getConeHeightInches() {
            return height;
        }

        double getCubeHeightInches() {
            return height - (coneChange ? kConeHangInches : 0);
        }
    }

    public enum Pieces {
        CONE,
        CUBE;


    }

    public Lift() {
        liftMotor = new CANSparkMax(Constants.ELEVATOR_LIFT_MOTOR_ID, MotorType.kBrushless);
        liftCtrl = liftMotor.getPIDController();
        liftEncoder = liftMotor.getEncoder();

        // PID Constants taken from 1885 2019 Elevator
        liftCtrl.setP(0.0005, PID_UP_SLOT);
        liftCtrl.setI(0.0, PID_UP_SLOT);
        liftCtrl.setD(0.0, PID_UP_SLOT);
        liftCtrl.setFF(0.000391419, PID_UP_SLOT);

        liftCtrl.setP(0.0005, PID_DOWN_SLOT);
        liftCtrl.setI(0.0, PID_DOWN_SLOT);
        liftCtrl.setD(0.0, PID_DOWN_SLOT);
        liftCtrl.setFF(0.000391419, PID_DOWN_SLOT);

        // Figure out how to convert inches per second to RPM and inches per second per second to RPM per second

        liftCtrl.setSmartMotionMaxVelocity(MAX_VELOCITY_DOWN, PID_DOWN_SLOT);
        liftCtrl.setSmartMotionMaxVelocity(MAX_VELOCITY_UP, PID_UP_SLOT);

        liftCtrl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, PID_UP_SLOT);
        liftCtrl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, PID_UP_SLOT);
        liftCtrl.setSmartMotionMaxAccel(MAX_ACCELERATION_DOWN, PID_DOWN_SLOT);
        liftCtrl.setSmartMotionMaxAccel(MAX_ACCELERATION_UP, PID_UP_SLOT);

        // The actual stowed height is stored as the 'zero' for the lift
        liftEncoder.setPosition((float)inchesToMotorRotations(Heights.STOWED.getCubeHeightInches()));
        liftMotor.setIdleMode(IdleMode.kBrake);

        liftMotor.setSmartCurrentLimit(MAX_CURRENT);

        // Consider dividing the upper soft limit by the sin of the angle to get the actual upper max
        liftMotor.setSoftLimit(SoftLimitDirection.kForward, (float)inchesToMotorRotations(Heights.HIGH.getConeHeightInches()));
        liftMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)inchesToMotorRotations(Heights.STOWED.getCubeHeightInches()));
    }

    public void reset() {
        liftEncoder.setPosition((float)inchesToMotorRotations(Heights.STOWED.getCubeHeightInches()));
    }

    // Spin motor 10% of [Left Joystick Y Axis] value
    public void testPlan1(double pct) {
        liftMotor.set(pct/10);
        SmartDashboard.putNumber("Elevator/Height Inches", getLinearHeightInches());
    }

    // Goes to a height (5 inches)
    public void testPlan2() {
        liftCtrl.setReference(inchesToMotorRotations(5d), ControlType.kSmartMotion);
    }

    // Indicate whether the cone or cube is wanted and what height to lift to (BUT DO NOT LIFT)
    public void testPlan3(Heights height) {
        this.prevHeight = height;
        SmartDashboard.putString("Elevator/Desired Game Piece", piece.name());
        SmartDashboard.putString("Elevator/Desired Height", height.name());
    }

    // Go to different heights (stowed, mid, high) and indicate whether the elevator should pivot
    public void testPlan4(Heights height) {
        double desiredHeight = piece == Pieces.CONE ? height.getConeHeightInches() : height.getCubeHeightInches();

        SmartDashboard.putNumber("Elevator/Height Inches", getLinearHeightInches());
        SmartDashboard.putNumber("Elevator/Desired Height Inches", desiredHeight);
        SmartDashboard.putString("Elevator/Desired Height", height.name());
        SmartDashboard.putBoolean("Elevator/Should Pivot", height != Heights.STOWED);

        // If the desired height is higher than the previous height go to the up slot, otherwise go to down slot for PID
        if (height.ordinal() >= this.prevHeight.ordinal()) { 
            liftCtrl.setReference(desiredHeight, ControlType.kSmartMotion, PID_UP_SLOT);
        } else {
            liftCtrl.setReference(desiredHeight, ControlType.kSmartMotion, PID_DOWN_SLOT);
        }

        this.prevHeight = height;
    }

    public void setPieceMode(Pieces piece) {
        this.piece = piece;
    }

    private double inchesToMotorRotations(double inches) {
        return inches/kMotorRotationsToHeightInches;
    }

    private double getLinearHeightInches() {
        return liftEncoder.getPosition()*kMotorRotationsToHeightInches;
    }

    // Gets the height of the elevator accounting for the tilt
    private double getAbsoluteHeightInches(double tiltDegrees) {
        return getLinearHeightInches() * Math.sin(tiltDegrees);
    }
}