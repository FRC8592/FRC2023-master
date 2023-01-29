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

    private final int PID_UP_SLOT = 0;
    private final int PID_DOWN_SLOT = 1;

    private final double MAX_ACCELERATION_UP = 1d; // 12 inches per second per second
    private final double MAX_ACCELERATION_DOWN = 1d;

    private final double MAX_VELOCITY_UP = 1d; // 24 inches per second
    private final double MAX_VELOCITY_DOWN = 1d;

    private final float LOWER_SOFT_LIMIT = 0;
    private final float UPPER_SOFT_LIMIT = 3;

    private final int kPulleyDiameterInches = 2;
    private final double kMotorRotationsToHeightInches = Constants.LIFT_GEARBOX_RATIO * 2 * Math.PI * kPulleyDiameterInches;

    private static final double kConeHangInches = 3; // The distance the cone hangs from the bottom of the intake
    private static final double kDropHeightInches = 6; // The height above the top of the node we want to drop from

    private Heights height = Heights.STOWED;

    public enum Heights {
        STOWED(kConeHangInches + kDropHeightInches, false),
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

    public Lift() {
        liftMotor = new CANSparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        liftCtrl = liftMotor.getPIDController();
        liftEncoder = liftMotor.getEncoder();

        // PID Constants taken from 1885 2019 Elevator

        liftCtrl.setP(0.0005, PID_UP_SLOT);
        liftCtrl.setI(0.0, PID_UP_SLOT);
        liftCtrl.setD(0.0, PID_UP_SLOT);
        liftCtrl.setFF(0.000391419, PID_UP_SLOT);

        liftCtrl.setP(0.0005, PID_DOWN_SLOT);
        liftCtrl.setI(0.0, PID_DOWN_SLOT);
        liftCtrl.setD(0.000391419, PID_DOWN_SLOT);

        liftCtrl.setSmartMotionMaxVelocity(MAX_VELOCITY_DOWN, PID_DOWN_SLOT);
        liftCtrl.setSmartMotionMaxVelocity(MAX_VELOCITY_UP, PID_UP_SLOT);

        liftCtrl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, PID_UP_SLOT);
        liftCtrl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, PID_UP_SLOT);
        liftCtrl.setSmartMotionMaxAccel(MAX_ACCELERATION_DOWN, PID_DOWN_SLOT);
        liftCtrl.setSmartMotionMaxAccel(MAX_ACCELERATION_UP, PID_UP_SLOT);

        liftMotor.setSoftLimit(SoftLimitDirection.kForward, UPPER_SOFT_LIMIT);
        liftMotor.setSoftLimit(SoftLimitDirection.kReverse, LOWER_SOFT_LIMIT);

        liftMotor.setIdleMode(IdleMode.kBrake);

        liftEncoder.setPosition(0.0);
    }

    public void reset() {
        liftEncoder.setPosition(0.0);
    }

    // Test motor spin (10% of actual input) and log height into smart dashboard
    public void testPlan1(double pct) {
        liftMotor.set(pct/10);
        SmartDashboard.putNumber("Elevator/Height Inches", getLinearHeightInches());
    }

    // Goes to a height (5 inches)
    public void testPlan2() {
        liftCtrl.setReference(inchesToMotorRotations(5d), ControlType.kSmartMotion);
    }

    // Indicate whether the cone or cube is wanted and what height to lift to
    public void testPlan3(Heights height, boolean cone) {
        this.height = height;
        SmartDashboard.putBoolean("Elevator/Desired Game Piece", cone);
        SmartDashboard.putString("Elevator/Desired Height", height.name());
    }

    // Go to different heights (stowed, mid, high) and indicate whether the elevator should pivot
    public void testPlan4(Heights height, boolean cone) {
        this.height = height;
        double desiredHeight = cone ? height.getConeHeightInches() : height.getCubeHeightInches();

        SmartDashboard.putNumber("Elevator/Height Inches", getLinearHeightInches());
        SmartDashboard.putNumber("Elevator/Desired Height Inches", desiredHeight);
        SmartDashboard.putString("Elevator/Desired Height", height.name());
        SmartDashboard.putBoolean("Elevator/Should Pivot", height != Heights.STOWED);

        liftCtrl.setReference(desiredHeight, ControlType.kSmartMotion);
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