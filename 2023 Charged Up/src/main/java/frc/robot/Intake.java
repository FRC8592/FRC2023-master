package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

public class Intake {
    private CANSparkMax rollerMotor;
    DigitalInput beamBreak;

    PIDController motorPID;

    public Intake() {
        rollerMotor = new CANSparkMax(Constants.ROLLER_MOTOR, MotorType.kBrushless);
        //PID constants WIP
        motorPID = new PIDController(0.01, 0, 0);
        beamBreak = new DigitalInput(Constants.BEAM_BREAK);
    }

    // Active rollers and stop once an item is collected
    public void intake() {
        if(!beamBreak.get()) {
            rollerMotor.set(motorPID.calculate(rollerMotor.get(), 0.25));
        }
    }

    // Reverse rollers to outtake an object
    public void outtake() {
        rollerMotor.set(motorPID.calculate(rollerMotor.get(), -0.25));
    }
}
