package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Claw {

    private DoubleSolenoid solenoid;

    public Claw() {
        //ADD FORWARDS/REVERSE CHANNEL VALS
        solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0);
    }

    public void openClaw() {
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void closeClaw() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }
}