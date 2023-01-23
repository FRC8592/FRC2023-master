package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Claw {
    private Solenoid solenoid;
    private Compressor comp;

    public Claw() {
        // CHANNEL VALUES NEED TUNING
        solenoid = new Solenoid(Constants.PNEUMATIC_MODULE_ID, PneumaticsModuleType.REVPH, Constants.CLAW_PNEUMATIC_CHANNEL);
        comp = new Compressor(Constants.PNEUMATIC_MODULE_ID, PneumaticsModuleType.REVPH);
        comp.enableAnalog(Constants.MIN_COMPRESSOR_PSI, Constants.MAX_COMPRESSOR_PSI);
    }

    public void openClaw() {
        solenoid.set(true);
    }

    public void closeClaw() {
        solenoid.set(false);
    }
}
