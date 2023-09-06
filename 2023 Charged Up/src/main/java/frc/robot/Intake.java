package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
    private CANSparkMax wristMotor;
    public CANSparkMax rollerMotor;
    private SparkMaxPIDController wristCtrl;
    private SparkMaxPIDController rollerCtrl;
    private RelativeEncoder wristEncoder;
    private RelativeEncoder rollerEncoder;
    private BeamSensor beamCone;
    private BeamSensor beamCube;
    private Timer coneTimer;

    public void logBeamBreaks(){
        // SmartDashboard.putBoolean("Cone Beam Break", !beamCone.isBroken());
        // SmartDashboard.putBoolean("Cube Beam Break", !beamCube.isBroken());
    }

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
        rollerMotor.setIdleMode(IdleMode.kBrake);

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

        beamCone = new BeamSensor(5);
        beamCube = new BeamSensor(6);
        coneTimer = new Timer();

        // SmartDashboard.putNumber("Wrist Desired Rotations", Constants.WRIST_INTAKE_ROTATIONS);
    }

    public void reset() {
        rollerEncoder.setPosition(0);
        wristEncoder.setVelocityConversionFactor(0);
    }

    public void writeToSmartDashboard() {
        double rawWristPosition = wristEncoder.getPosition(); // rotations
        
        // SmartDashboard.putNumber("Wrist position", rawWristPosition);
        // SmartDashboard.putNumber("Wrist velocity", wristEncoder.getVelocity());
        // SmartDashboard.putNumber("Wrist current", wristMotor.getOutputCurrent());
        // SmartDashboard.putNumber("Roller current", rollerMotor.getOutputCurrent());
        // SmartDashboard.putNumber("Roller velocity", rollerEncoder.getVelocity());
        // SmartDashboard.putBoolean("Cone beam broken", beamCone.isBroken());
        // SmartDashboard.putBoolean("Cube beam broken", beamCube.isBroken());
    }

    public void intakeRoller() {
        // double currentTime = 0;
        // if (beamCone.isBroken() && beamCube.isBroken()) {
        //     coneTimer.start();
        //     spinRollers(0.8);
        // }else if (!beamCone.isBroken()){
        //     currentTime = coneTimer.get();
        //     if (coneTimer.get() - currentTime >= 0.75){

        //         spinRollers(0.0);
        //         coneTimer.reset();
        //         coneTimer.stop();
        //     }
        // }else{
        //     spinRollers(0.0);
        //     coneTimer.reset();
        //     coneTimer.stop();
        // }
        // rollerMotor.set(0.7);
        // rollerCtrl.setReference(1000, ControlType.kSmartVelocity);

        spinRollers(0.7);
    }

    public void coneIntakeRoller(){
        // rollerCtrl.setReference(1000, ControlType.kSmartVelocity);
        // rollerMotor.set(0.7);
        rollerMotor.setSmartCurrentLimit(30);
        spinRollers(0.7);
    }

    public void cubeIntakeRoller(){
        // rollerCtrl.setReference(1000, ControlType.kSmartVelocity);
        rollerMotor.setSmartCurrentLimit(60);
        spinRollers(0.7);
    }

    public void outtakeRoller() {
        // rollerMotor.set(-0.8);
        // rollerCtrl.setReference(-1000, ControlType.kSmartVelocity);
        rollerMotor.setSmartCurrentLimit(40);
        spinRollers(-1.0);
    }

    public void scoreRoller() {
        if(Math.abs(wristEncoder.getPosition() - Constants.WRIST_SCORING_ROTATIONS) <= 3.0){
            outtakeRoller();
        } else {
            stopRoller();
        }
    }

    public void spinRollers(double pct) {
        rollerCtrl.setReference(pct*Constants.ROLLER_MAX_VELOCITY, ControlType.kSmartVelocity);
        // rollerMotor.set(pct);
        // double currentTime = 0;
        // if (beamCone.isBroken() && beamCube.isBroken()) {
        //     coneTimer.start();
        //     rollerMotor.set(pct);
        // }else if (!beamCone.isBroken()){
        //     currentTime = coneTimer.get();
        //     if (coneTimer.get() - currentTime >= 0.75){

        //         rollerMotor.set(0.0);
        //         coneTimer.reset();
        //         coneTimer.stop();
        //     }
        // }else{
        //     rollerMotor.set(0.0);
        //     coneTimer.reset();
        //     coneTimer.stop();
        // }
    }

    public void throwPiece() {
        setWrist(Constants.WRIST_INTAKE_ROTATIONS);
        double rawWrist = wristEncoder.getPosition();
        if (Math.abs(rawWrist) >= Constants.WRIST_INTAKE_ROTATIONS / 5) {
            outtakeRoller();
        }
    }

    public void enableWrist(boolean enable) {
        if (enable) {
            wristCtrl.setReference(SmartDashboard.getNumber("Wrist Desired Rotations", Constants.WRIST_INTAKE_ROTATIONS), ControlType.kSmartMotion);
        } else {
            wristCtrl.setReference(0.0, ControlType.kSmartMotion);
        }
    }

    public void setWrist(double rotations) {
        wristCtrl.setReference(rotations, ControlType.kSmartMotion, 0, 0.01);
    }

    public void stopRoller() {
        rollerMotor.set(0.0);
    }

    public boolean hasPiece() { // Beam break not tested yet
        return beamCone.isBroken() || beamCube.isBroken();
    }

    public void haltWrist() {
        double rawWrist = wristEncoder.getPosition();
        setWrist(rawWrist);
    }
}