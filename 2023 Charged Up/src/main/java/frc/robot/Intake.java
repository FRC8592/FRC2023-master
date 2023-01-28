package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

public class Intake {
    private CANSparkMax rollerMotor;
    BeamBreak beamBreak;

    SparkMaxPIDController motorPID;

    public Intake() {
        rollerMotor = new CANSparkMax(Constants.ROLLER_MOTOR, MotorType.kBrushless);
        //PID constants WIP
        motorPID = rollerMotor.getPIDController();
        motorPID.setP(0.01);
        motorPID.setI(0);
        motorPID.setD(0);
        beamBreak = new BeamBreak(Constants.BEAM_BREAK, 0.1);
    }

    // Active rollers and stop once an item is collected
    public void intakeCube() {
        if(!beamBreak.isBroken()) {
            // motorPID.setReference(0.25, ControlType.kVelocity);
            rollerMotor.set(0.1);
        }
    }

    // Reverse rollers to intake cube
    public void intakeCone() {
        // motorPID.setReference(-0.25, ControlType.kVelocity);
        rollerMotor.set(-0.1);
    }
}
