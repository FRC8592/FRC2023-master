package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
    private CANSparkMax wristMotor;
    private CANSparkMax rollerMotor;
    private SparkMaxPIDController wristCtrl;
    private SparkMaxPIDController rollerCtrl;
    private RelativeEncoder wristEncoder;
    private RelativeEncoder rollerEncoder;
    private BeamSensor beam;

    public Intake() {
        rollerMotor = new CANSparkMax(Constants.ROLLER_ID, MotorType.kBrushless);
        rollerEncoder = rollerMotor.getEncoder();
        rollerCtrl = rollerMotor.getPIDController();
        rollerMotor.setSmartCurrentLimit(Constants.ROLLER_MAX_CURRENT_AMPS);
        rollerMotor.setIdleMode(IdleMode.kCoast);

        wristMotor = new CANSparkMax(Constants.WRIST_ID, MotorType.kBrushless);
        wristEncoder = wristMotor.getEncoder();
        wristCtrl = wristMotor.getPIDController();
        wristMotor.setSmartCurrentLimit(Constants.WRIST_MAX_CURRENT_AMPS);
        wristMotor.setIdleMode(IdleMode.kBrake);

        rollerCtrl.setP(Constants.ROLLER_KP);
        rollerCtrl.setI(Constants.ROLLER_KI);
        rollerCtrl.setD(Constants.ROLLER_KD);
        rollerCtrl.setFF(Constants.ROLLER_KF);
        rollerCtrl.setSmartMotionMaxVelocity(Constants.ROLLER_MAX_VELOCITY, Constants.ROLLER_PID_SLOT);
        rollerCtrl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, Constants.ROLLER_PID_SLOT);
        rollerCtrl.setSmartMotionMaxAccel(Constants.ROLLER_MAX_ACCELERATION, Constants.ROLLER_PID_SLOT);

        wristCtrl.setP(Constants.WRIST_KP);
        wristCtrl.setI(Constants.WRIST_KI);
        wristCtrl.setD(Constants.WRIST_KD);
        wristCtrl.setFF(Constants.WRIST_KF);
        wristCtrl.setSmartMotionMaxVelocity(Constants.WRIST_MAX_VELOCITY, Constants.WRIST_PID_SLOT);
        wristCtrl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, Constants.WRIST_PID_SLOT);
        wristCtrl.setSmartMotionMaxAccel(Constants.WRIST_MAX_ACCELERATION, Constants.WRIST_PID_SLOT);

        beam = new BeamSensor(Constants.BEAM_BREAK_ID);
    }

    public void reset() {
        rollerCtrl.setReference(0,ControlType.kSmartVelocity);
        wristCtrl.setReference(0,ControlType.kSmartVelocity);

        rollerEncoder.setPosition(0);
        wristEncoder.setVelocityConversionFactor(0);
    }

    public void writeToSmartDashboard() {
        double rawWristPosition = wristEncoder.getPosition(); // rotations
        double rawWristVelocity = wristEncoder.getVelocity(); // RPM
        double rawRollerVelocity = rollerEncoder.getVelocity(); // RPM
        
        SmartDashboard.putNumber("Wrist/Position (Degrees)", rawWristPosition * 360.0 * Constants.WRIST_GEAR_RATIO);
        SmartDashboard.putNumber("Wrist/Velocity (Degrees per Second)", rawWristVelocity * 60.0 * Constants.WRIST_GEAR_RATIO);
        SmartDashboard.putNumber("Roller/Velocity (Degrees per Second)", rawRollerVelocity * 60.0 * Constants.ROLLER_GEAR_RATIO);
    }

    public void testPlan1() {

    }

    public void testPlan2() {
        
    }

    public void testPlan3() {
        
    }
}