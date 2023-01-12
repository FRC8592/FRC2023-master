//////////////////////////////////////////////////////////////////////////////////////////////////////
// Power measurement and control
/////////////////////////////////////////////////////////////////////////////////////////////////////

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class Power {
    // Object variables
    private PowerDistribution revPDH;

    //
    // Shared power variables
    //
    public double temp;
    public double voltage;
    public double current;
    public double power;
    public double energy;

    // Shuffleboard for power data
    ShuffleboardTab powerTab;


    //
    // Constructor for power control
    //
    public Power() {
        // Create new Rev Power Distribution object
       // revPDH = new PowerDistribution(Constants.PDH_CAN, PowerDistribution.ModuleType.kRev);
        
        // Create the shuffleboard tab for power data
        powerTab = Shuffleboard.getTab("Power");

    }


    //
    // Periodically post power data to the dashboard
    //
    public void powerPeriodic() {
        // Get parameters from the PDH
        /*
        temp    = revPDH.getTemperature();
        voltage = revPDH.getVoltage();
        current = revPDH.getTotalCurrent();
        power   = revPDH.getTotalPower();
        energy  = revPDH.getTotalEnergy();
        */

        // Place all parameters onto a dedicated Shuffleboard tab
        // Shuffleboard.selectTab("Power");

        // SmartDashboard.putNumber("Temperature", temp);
        // SmartDashboard.putNumber("Voltage", voltage);
        // SmartDashboard.putNumber("Current", current);
        // SmartDashboard.putNumber("Power", power);
        // SmartDashboard.putNumber("Energy", energy);

        // Shuffleboard.selectTab("SmartDashboard");   // Switch back to default
    }


    //
    // Turn the switchable 12v port on
    //
    public void relayOn() {
        // revPDH.setSwitchableChannel(true);
    }


    //
    // Turn the switchable 12v port off
    //
    public void relayOff() {
        // revPDH.setSwitchableChannel(false);
    }
}