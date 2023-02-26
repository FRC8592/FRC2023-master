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
    private BeamSensor beamCone;
    private BeamSensor beamCube;

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

        rollerCtrl.setP(Constants.ROLLER_KP, 0);
        rollerCtrl.setI(Constants.ROLLER_KI, 0);
        rollerCtrl.setD(Constants.ROLLER_KD, 0);
        rollerCtrl.setFF(Constants.ROLLER_KF, 0);
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

        wristEncoder.setPosition(0);

        beamCone = new BeamSensor(Constants.BEAM_BREAK_CONE_ID);
        beamCube = new BeamSensor(Constants.BEAM_BREAK_CUBE_ID);

        SmartDashboard.putNumber("Wrist Desired Rotations", Constants.WRIST_INTAKE_ROTATIONS);
    }

    public void reset() {
        rollerEncoder.setPosition(0);
        wristEncoder.setVelocityConversionFactor(0);
    }

    public void writeToSmartDashboard() {
        double rawWristPosition = wristEncoder.getPosition(); // rotations
        
        SmartDashboard.putNumber("Wrist position", rawWristPosition);
        SmartDashboard.putNumber("Wrist current", wristMotor.getOutputCurrent());
        SmartDashboard.putNumber("Roller current", rollerMotor.getOutputCurrent());
        SmartDashboard.putNumber("Roller velocity", rollerEncoder.getVelocity());
        SmartDashboard.putBoolean("Cone beam broken", beamCone.isBroken());
        SmartDashboard.putBoolean("Cone beam broken", beamCube.isBroken());
    }

    public void intakeRoller() {
        if (!beamCone.isBroken() && !beamCube.isBroken()) {
            rollerMotor.set(0.8);
        } else {
            rollerMotor.set(0.0);
        }
        // rollerMotor.set(0.8);
    }

    public void outtakeRoller() {
        rollerMotor.set(-0.8);
    }

    public void scoreRoller() {
        if(Math.abs(wristEncoder.getPosition() - Constants.WRIST_SCORING_ROTATIONS) <= 5.0){
            outtakeRoller();
        } else {
            stopRoller();
        }

        SmartDashboard.putNumber("Wrist Error", Math.abs(wristEncoder.getPosition() - Constants.WRIST_SCORING_ROTATIONS));
    }

    public void spinRollers(double pct) {
        rollerMotor.set(pct);
    }

    public void enableWrist(boolean enable) {
        if (enable) {
            wristCtrl.setReference(SmartDashboard.getNumber("Wrist Desired Rotations", Constants.WRIST_INTAKE_ROTATIONS), ControlType.kSmartMotion);
        } else {
            wristCtrl.setReference(0.0, ControlType.kSmartMotion);
        }
    }

    public void stopRoller() {
        rollerMotor.set(0.0);
    }

    public boolean hasPiece() { // Beam break not tested yet
        return beamCone.isBroken() || beamCube.isBroken();
    }
}