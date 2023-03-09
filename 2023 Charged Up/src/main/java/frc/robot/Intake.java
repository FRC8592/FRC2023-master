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
    private CANSparkMax rollerMotor;
    private SparkMaxPIDController wristCtrl;
    private SparkMaxPIDController rollerCtrl;
    private RelativeEncoder wristEncoder;
    private RelativeEncoder rollerEncoder;
    private BeamSensor beamCone;
    private BeamSensor beamCube;
    private Timer coneTimer;

    public void logBeamBreaks(){
        SmartDash.putBoolean("Cone Beam Break", !beamCone.isBroken(), false);
        SmartDash.putBoolean("Cube Beam Break", !beamCube.isBroken(), false);
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

        beamCone = new BeamSensor(Constants.BEAM_BREAK_CONE_ID);
        beamCube = new BeamSensor(Constants.BEAM_BREAK_CUBE_ID);
        coneTimer = new Timer();

        SmartDash.putNumber("Wrist Desired Rotations", Constants.WRIST_INTAKE_ROTATIONS, false);
    }

    public void reset() {
        rollerEncoder.setPosition(0);
        wristEncoder.setVelocityConversionFactor(0);
    }

    public void writeToSmartDashboard() {
        double rawWristPosition = wristEncoder.getPosition(); // rotations
        
        SmartDash.putNumber("Wrist position", rawWristPosition, false);
        SmartDash.putNumber("Wrist current", wristMotor.getOutputCurrent(), false);
        SmartDash.putNumber("Roller current", rollerMotor.getOutputCurrent(), false);
        SmartDash.putNumber("Roller velocity", rollerEncoder.getVelocity(), false);
        SmartDash.putBoolean("Cone beam broken", beamCone.isBroken(), false);
        SmartDash.putBoolean("Cone beam broken", beamCube.isBroken(), false);
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
        rollerMotor.set(0.7);
        // spinRollers(0.8);
    }

    public void coneIntakeRoller(){
        rollerMotor.setSmartCurrentLimit(80);
        rollerMotor.set(0.7);
    }

    public void cubeIntakeRoller(){
        rollerMotor.setSmartCurrentLimit(60);
        rollerMotor.set(0.7);
    }

    public void outtakeRoller() {
        // rollerMotor.set(-0.8);
        rollerMotor.setSmartCurrentLimit(40);
        rollerMotor.set(-1.0);
    }

    public void scoreRoller() {
        if(Math.abs(wristEncoder.getPosition() - Constants.WRIST_SCORING_ROTATIONS) <= 5.0){
            outtakeRoller();
        } else {
            stopRoller();
        }

        SmartDash.putNumber("Wrist Error", Math.abs(wristEncoder.getPosition() - Constants.WRIST_SCORING_ROTATIONS), false);
    }

    public void spinRollers(double pct) {
        rollerMotor.set(pct);
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
}