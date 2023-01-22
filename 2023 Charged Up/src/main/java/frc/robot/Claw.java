package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Claw {

    //robot will use 1 double solenoid and 3 single
    private Solenoid solenoid;
    private Compressor comp;

    public Claw() {
        //Make sure channel values are correct
        solenoid = new Solenoid(14, PneumaticsModuleType.REVPH, 15);
        comp = new Compressor(14, PneumaticsModuleType.REVPH);
        comp.enableAnalog(60, 100);
    }

    public void openClaw() {
        solenoid.set(true);
        SmartDashboard.putBoolean("Activated", true);
    }

    public void closeClaw() {
        solenoid.set(false);
        SmartDashboard.putBoolean("Activated", false);
    }
}