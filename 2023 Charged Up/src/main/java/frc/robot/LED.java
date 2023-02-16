package frc.robot;

import java.awt.Color;

import javax.xml.validation.SchemaFactory;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class LED {
    
    private AddressableLED liftNEOPIXELS;
    private AddressableLEDBuffer liftBuffer;
    private Timer timer;
    private Timer blinkSpeedTimer;
    private BlinkSpeed blinkSpeed = BlinkSpeed.SOLID;

    private int count = 0;
    private double brightnessMultiplier = 1;

    final int LED_LENGTH = 43;

    /**
     * Premade color presets
     */
    public enum Color {
        RED (200, 0, 0),
        GREEN (0, 255, 0),
        CYAN (0, 200, 255),
        BLUE (0, 0, 255),
        YELLOW (255, 128, 0),
        PURPLE (138,43,226),
        ORANGE (243, 50, 0),
        WHITE (255, 255, 255),
        OFF (0, 0, 0);
    
        public final int red;
        public final int blue;
        public final int green;
    
        Color(int red, int green, int blue){
    
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
    
    }

    /**
     * Blink speed presets
     */
    public enum BlinkSpeed {
        SLOW (2.0),
        NORMAL (1.0),
        SOLID (0.0);

        public final double speed;

        BlinkSpeed(double speed) {
            this.speed = speed;
        }
    }

    public LED(){
        liftNEOPIXELS = new AddressableLED(0);
        liftBuffer = new AddressableLEDBuffer(LED_LENGTH);
        liftNEOPIXELS.setLength(LED_LENGTH);
        timer = new Timer();
        blinkSpeedTimer = new Timer();
    }

    /**
     * Set the current leds to a certain color
     * 
     * @param i         index of the led light to set
     * @param color     color to set the light
     */
    public void setColor(int i, Color color){
        blinkSpeedTimer.start();
        SmartDashboard.putNumber("blinkspeedtimer", blinkSpeedTimer.get());
        if(blinkSpeedTimer.hasElapsed(blinkSpeed.speed / 2.0)) {
            SmartDashboard.putBoolean("blinking", true);
            liftBuffer.setRGB(i, (int)(color.red * brightnessMultiplier), (int)(color.green * brightnessMultiplier), (int)(color.blue * brightnessMultiplier));   
            
            if (blinkSpeedTimer.hasElapsed(blinkSpeed.speed)) {
                blinkSpeedTimer.reset();
            }
        } else {
            SmartDashboard.putBoolean("blinking", false);
            liftBuffer.setRGB(i, 0, 0, 0);  
        }
    }

    /**
     * Set a percentage of the LEDs on, automatically spaces out lights
     * 
     * @param pct   the percentage of LEDs to turn on (0 - 100)
     * @param color the color of the LEDs that are on 
     */
    public void setPct(double pct, Color color) {
        // loop through LEDs, and set the passed percentage as on
        for (int ledIndex = 0; ledIndex < LED_LENGTH; ledIndex++){
            if (pct != 0 && (double)ledIndex % (1.0 / (pct / 100.0)) < 1.0){
                setColor(ledIndex, color);
            }else {
                setColor(ledIndex, Color.OFF);
            }
        }
        liftNEOPIXELS.setData(liftBuffer);
        liftNEOPIXELS.start(); 
    }

    /**
     * Set the pulsing state of the robot
     * 
     * @param speed     blink speed of the robot
     */
    public void setPulseState(BlinkSpeed speed) {
        blinkSpeed = speed;
    }

    /**
     * Run two colors up and down the LED strip
     * 
     * @param colorA    first color
     * @param colorB    second color
     */
    public void upAndDown(Color colorA, Color colorB){
        timer.start();
            if(timer.get() >= .05){
                
                count++;
                timer.reset();
                if(count > LED_LENGTH){
                    count = 0;
                }
                for(int i = 0; i < LED_LENGTH; i++){
                    if (Math.sin(1 * (double)(i + count))  > 0){
                        setColor(i, colorA);
                    }
               /*     else if(Math.sin((double)(i + count)) > -.5){
                        liftBuffer.setRGB(i, 0, 255, 0);
                    } */
                     else  {
                        setColor(i, colorB);
                    }
                }
            }
        liftNEOPIXELS.setData(liftBuffer);
        liftNEOPIXELS.start();
        
    }

    /** 
     * Turn the LEDs off
     */
    public void turnOff() {
        setPct(100, Color.OFF);
    }

    /**
     * Set more LEDs on depending on how close you are to a target, when target is "in range" LEDs turn green
     * 5m = max distance
     * 0.75m = closest distance ("in range")
     * 
     * @param distToTarget  the distance to the target (meters)
     */
    public void setProximity(double distToTarget) {
        Color color = Color.RED;
        // Formula to calculate how many LEDs to set in each strip
        // max = max distance before LEDs start lighting up
        // min = distance until piece is "in range"
        double max = 5.0;
        double min = 0.75;
        double difference = max - min;
        int numLEDs = (int)((max - distToTarget) / difference * (LED_LENGTH / 2 - 0.5));
        
        //If distance is at or closer to the max distance, set the color of LEDs to green and cap amount to turn on
        if (numLEDs >= LED_LENGTH / 2) {
            numLEDs = LED_LENGTH / 2;
            color = Color.GREEN;
        }

        // loop through one side of the LEDs and set an amount of LEDs on depending on distance
        for (int ledIndex = 0; ledIndex < LED_LENGTH / 2; ledIndex++){
            if(ledIndex < numLEDs) {
                setColor(ledIndex, color);
                setColor((LED_LENGTH - 1) - ledIndex, color);
            } else {
                setColor(ledIndex, Color.OFF);
                setColor((LED_LENGTH - 1) - ledIndex, Color.OFF);
            }
        }
        liftNEOPIXELS.setData(liftBuffer);
        liftNEOPIXELS.start();
    }

    /**
     * Sets the brightness level for LEDS
     * 
     * @param pct percent of brightness (0 - 100)
     */
    public void setBrightness(double pct) {
        brightnessMultiplier = pct / 100.0;
    }

    /**
     * Check the voltage of the robot, and if lower than a certain value for a certain time, blink a slow red
     */
    // private double voltages[] = new double[10];
    // int counter = 0, sum = 0, avg;
    public void checkVoltage() {
        // SmartDashboard.putNumber("avg volt", avg);
        double voltage = RoboRioDataJNI.getVInVoltage();
        SmartDashboard.putNumber("voltage", voltage);
        // voltages[counter] = voltage;
        // sum += voltage;
        // counter = (counter + 1) % voltages.length;

        // if (counter == voltages.length - 1) {
        //     avg = sum / counter;
        //     sum = 0;
        // }

        // if(avg != 0.0 && avg < Constants.MINIMUM_VOLTAGE) {
        //     setPulseState(BlinkSpeed.SLOW);
        //     setPct(100, Color.RED);
        //     SmartDashboard.putString("blink status", blinkSpeed.name());
        // }
        if (voltage < 9.0) {
            SmartDashboard.putBoolean("this joint ran", true);
            setPulseState(BlinkSpeed.SLOW);
            setPct(100, Color.RED);
        } else {
            SmartDashboard.putBoolean("this joint ran", false);
        }
    }

    private boolean first = true;
    private double valY = 0, valO = 0, valR = 0; 
    private double numYellow, numOrange, numRed, total, prevTotal = 0, difference;
    private Timer resetTimer = new Timer();

    public void setFire(Color colorA, Color colorB, Color colorC) {
        blinkSpeed = BlinkSpeed.SOLID;
        SmartDashboard.putNumber("Num Yellow", numYellow);
        SmartDashboard.putNumber("Num Orange", numOrange);
        SmartDashboard.putNumber("Num Red", numRed);

        SmartDashboard.putNumber("sin val", Math.sin(valY));
        resetTimer.start();
        if (first) {
            first = false;
            numYellow = (Math.random() * 5); //6
            numOrange = Math.abs(numYellow + (Math.random() * 7 - 3)); //6 + 5 = 11
            numRed = Math.abs(numOrange + (Math.random() * 7 - 5)); // 11 + 5 = 16
        } else {
            valY = (valY + Math.random() * 1.0) % (2 * Math.PI);
            valO = (valO + Math.random() * 2.0) % (2 * Math.PI);
            valR = (valR + Math.random() * 2.5) % (2 * Math.PI);
            numYellow +=  3.0 * Math.sin(valY);
            numOrange += 3.0 * Math.sin(valO);
            numRed += 3.5 * Math.sin(valR);
        }

        if (resetTimer.get() > 5) {
            first = true;
            resetTimer.reset();
        }

        numYellow = Math.min(5, Math.max(3, numYellow));
        numOrange = Math.min(9, Math.max(4, numOrange));
        numRed = Math.min(11, Math.max(5, numRed));

        total = numYellow +  numOrange + numRed;
        difference = total - prevTotal;
        prevTotal = total;
        if(difference > 2) {
            numRed -= difference / 4.0;
            numOrange -= difference / 3.0;
            numYellow -= difference / 2.0;
        }

        for(int i = 0; i < LED_LENGTH / 2; i++) {
            if(i < (int)Math.abs(numYellow)) {

                setColor(i, colorA);
                setColor((LED_LENGTH - 1) - i, colorA);
            } else if (i < (int)Math.abs(numYellow) + (int)Math.abs(numOrange)) {

                setColor(i, colorB);
                setColor((LED_LENGTH - 1) - i, colorB);
            } else if (i < (int)Math.abs(numYellow) + (int)Math.abs(numOrange) + (int)Math.abs(numRed)) {

                setColor(i, colorC);
                setColor((LED_LENGTH - 1) - i, colorC);
            } else {
                setColor(i, Color.OFF);
                setColor((LED_LENGTH - 1) - i, Color.OFF);
            }
        }

        // if(ledIndex < numLEDs) {
        //     setColor(ledIndex, color);
        //     setColor((LED_LENGTH - 1) - ledIndex, color);
        // } else {
        //     setColor(ledIndex, Color.OFF);
        //     setColor(LED_LENGTH - 1 - ledIndex, Color.OFF);
        // }
        liftNEOPIXELS.setData(liftBuffer);
        liftNEOPIXELS.start();
    }
}