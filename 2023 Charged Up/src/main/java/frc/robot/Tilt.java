package frc.robot;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

public class Tilt {
    private CANSparkMax tiltMotor;
    private RelativeEncoder tiltEncoder;
    private SparkMaxPIDController tiltCtrl;

    public enum Angle {
        IN(0),
        OUT(60);

        private double angle;
        Angle(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return angle;
        }

        public double toMotorRotations() { // Find a way to convert between motor rotations and actual angle
            return angle;
        }
    }

    private final int TILT_PID_SLOT = 0;

    private final double MAX_ACCEL = 0.0; // Degrees per second per second -> RPM per second
    private final double MAX_VEL = 0.0; // Degrees per second -> RPM
    private final int CURRENT_LIMIT = 30; // Amps

    public Tilt() {
        tiltMotor = new CANSparkMax(Constants.ELEVATOR_TILT_MOTOR_ID, MotorType.kBrushless);
        tiltEncoder = tiltMotor.getEncoder();
        tiltCtrl = tiltMotor.getPIDController();
        
        tiltCtrl.setP(0.00001);
        tiltCtrl.setI(0.0);
        tiltCtrl.setD(0.0);
        tiltCtrl.setFF(0.0);

        tiltCtrl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, TILT_PID_SLOT);
        tiltCtrl.setSmartMotionMaxAccel(MAX_ACCEL, TILT_PID_SLOT);
        tiltCtrl.setSmartMotionMaxVelocity(MAX_VEL, TILT_PID_SLOT);
        tiltMotor.setSmartCurrentLimit(CURRENT_LIMIT);

        tiltMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Angle.OUT.toMotorRotations());
        tiltMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Angle.IN.toMotorRotations());
        
        tiltMotor.setIdleMode(IdleMode.kBrake);
        tiltEncoder.setPosition(Angle.IN.toMotorRotations());
    }

    // Spin 10% of [Right Y Axis] value
    public void testPlan1(double pct) {
        tiltMotor.set(pct/10);
    }

    public void testPlan2() {
        
    }

    public void testPlan3() {
        
    }

    public void testPlan4() {
        
    }

}
