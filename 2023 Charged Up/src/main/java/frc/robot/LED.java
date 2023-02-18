package frc.robot;

import java.awt.Color;

import javax.xml.validation.SchemaFactory;

import edu.wpi.first.hal.simulation.PowerDistributionDataJNI;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class LED {
    
    private AddressableLED liftNEOPIXELS;
    private AddressableLEDBuffer liftBuffer;
    private Timer timer;
    private Timer blinkSpeedTimer;
    private BlinkSpeed blinkSpeed = BlinkSpeed.SOLID;
    private LEDMode mode = LEDMode.OFF;
    // private Vision vision = new Vision(Constants.LIMELIGHT_BALL, Constants.BALL_LOCK_ERROR,
    // Constants.BALL_CLOSE_ERROR, Constants.BALL_CAMERA_HEIGHT, Constants.BALL_CAMERA_ANGLE, 
    // Constants.BALL_TARGET_HEIGHT, Constants.BALL_ROTATE_KP, Constants.BALL_ROTATE_KI, Constants.BALL_ROTATE_KD, new FRCLogger(true, "CustomLogs"));
    private Vision vision;

    private int count = 0;
    private double brightnessMultiplier = 1;
    private boolean lowVolts = false;
    private Timer delayTimer = new Timer();
    private Power power = new Power();

    final int LED_LENGTH = 43;

    /**
     * Premade color presets
     */
    public enum Color {
        RED (200, 0, 0),
        GREEN (0, 255, 0),
        CYAN (0, 160, 255),
        BLUE (0, 0, 255),
        YELLOW (255, 128, 0),
        PURPLE (138, 0,226),
        ORANGE (243, 100, 0),
        WHITE (255, 255, 255),
        BROWN (74, 23, 0),
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
        SLOW (1.0),
        NORMAL (0.5),
        SOLID (0.0);

        public final double speed;

        BlinkSpeed(double speed) {
            this.speed = speed;
        }
    }

    public enum LEDMode {
        CONE,
        CUBE,
        TARGETLOCK,
        STOPPLACING,
        ATTENTION,
        OFF;
    }

    public LED(PowerDistribution powerDist, Vision vision){
        liftNEOPIXELS = new AddressableLED(0);
        liftBuffer = new AddressableLEDBuffer(LED_LENGTH);
        liftNEOPIXELS.setLength(LED_LENGTH);
        timer = new Timer();
        blinkSpeedTimer = new Timer();
        this.vision = vision;
    }

    public void updatePeriodic() {
        power.powerPeriodic();
        SmartDashboard.putString("LED Mode", mode.name());
        
        if (power.voltage < 9.0 || lowVolts) {
            delayTimer.start();
            lowVolts = true;
            lowVoltage();
            if (delayTimer.get() > 5) {
                delayTimer.stop();
                lowVolts = false;
            } 
        } else {
            switch (mode) {
                case CONE:
                    blinkSpeed = BlinkSpeed.SOLID;
                    upAndDown(Color.PURPLE, Color.OFF);
                    break;
                case CUBE:
                    blinkSpeed = BlinkSpeed.SOLID;
                    upAndDown(Color.YELLOW, Color.OFF);
                    break;
                case TARGETLOCK:
                //5m = 196.85
                    blinkSpeed = BlinkSpeed.SOLID;
                    vision.updateVision();
                    /*
                    if (!vision.isTargetLocked()) {
                        delayTimer.start();
                        if (delayTimer.get() > 0.5) {
                            setPct(50, Color.ORANGE);
                        }
                    } else */if (vision.distanceToTarget() < 80 && vision.distanceToTarget() >= 0.0) {
                        delayTimer.start();
                        if (delayTimer.get() > 0.5) {
                            setProximity(vision.distanceToTarget() * Constants.INCHES_TO_METERS);
                        }
                    } else {
                        delayTimer.stop();
                    }
                    break;
                case STOPPLACING:
                    blinkSpeed = BlinkSpeed.SOLID;
                    upAndDown(Color.WHITE, Color.OFF);
                    break;
                case ATTENTION:
                    blinkSpeed = BlinkSpeed.SOLID;
                    upAndDown(Color.ORANGE, Color.BLUE);
                    break;
                case OFF:
                    blinkSpeed = BlinkSpeed.SOLID;
                    turnOff();
                    break;
            }
        }
    }

    /**
     * Set a preset state of the LEDs
     * 
     * @param mode      mode to set the LEDs to
     */
    public void setState(LEDMode mode) {
        this.mode = mode;
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
     * Sets the brightness level for LEDS
     * 
     * @param pct percent of brightness (0 - 100)
     */
    public void setBrightness(double pct) {
        brightnessMultiplier = pct / 100.0;
    }

    /**
     * Set the current leds to a certain color
     * 
     * @param i         index of the led light to set
     * @param color     color to set the light
     */
    private void setColor(int i, Color color){
        blinkSpeedTimer.start();
        if(blinkSpeedTimer.hasElapsed(blinkSpeed.speed)) {
            liftBuffer.setRGB(i, (int)(color.red * brightnessMultiplier), (int)(color.green * brightnessMultiplier), (int)(color.blue * brightnessMultiplier));   
            
            if (blinkSpeedTimer.hasElapsed(blinkSpeed.speed * 2.0)) {
                blinkSpeedTimer.reset();
            }
        } else {
            liftBuffer.setRGB(i, 0, 0, 0);  
        }
    }

    /**
     * Set a percentage of the LEDs on, automatically spaces out lights
     * 
     * @param pct   the percentage of LEDs to turn on (0 - 100)
     * @param color the color of the LEDs that are on 
     */
    private void setPct(double pct, Color color) {
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
     * Run two colors up and down the LED strip
     * 
     * @param colorA    first color
     * @param colorB    second color
     */
    private void upAndDown(Color colorA, Color colorB){
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
    private void turnOff() {
        setPct(100, Color.OFF);
    }

    /**
     * Set more LEDs on depending on how close you are to a target, when target is "in range" LEDs turn green
     * 5m = max distance
     * 0.75m = closest distance ("in range")
     * 
     * @param distToTarget  the distance to the target (meters)
     */
    private void setProximity(double distToTarget) {
        Color color = Color.RED;
        // Formula to calculate how many LEDs to set in each strip
        // max = max distance before LEDs start lighting up
        // min = distance until piece is "in range"
        double max = 2.0;
        double min = 0.75;
        double difference = max - min;
        int numLEDs = (int)((max - distToTarget) / difference * (LED_LENGTH / 2 - 0.5));
        
        //If distance is at or closer to the max distance, set the color of LEDs to green and cap amount to turn on
        if (numLEDs >= LED_LENGTH / 2) {
            numLEDs = LED_LENGTH / 2;
            color = Color.GREEN;
            timer.start();
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
     * Check the voltage of the robot, and if lower than a certain value for a certain time, blink a slow red
     */
    // private double voltages[] = new double[10];
    // int counter = 0, sum = 0, avg;
    private void lowVoltage() {
        setPulseState(BlinkSpeed.SLOW);
        setPct(100, Color.RED);
    }

    private boolean first = true;
    private double valY = 0, valO = 0, valR = 0; 
    private double numYellow, numOrange, numRed;

    private void setFire(Color colorA, Color colorB, Color colorC) {
        blinkSpeed = BlinkSpeed.SOLID;
        SmartDashboard.putNumber("Num Yellow", numYellow);
        SmartDashboard.putNumber("Num Orange", numOrange);
        SmartDashboard.putNumber("Num Red", numRed);

        SmartDashboard.putNumber("sin val", Math.sin(valY));
        if (first) {
            first = false;
            numYellow = (Math.random() * 5); //6
            numOrange = Math.abs(numYellow + (Math.random() * 7 - 3)); //6 + 5 = 11
            numRed = Math.abs(numOrange + (Math.random() * 7 - 5)); // 11 + 5 = 16
        } else {
            valY = (valY + Math.random() * 1.5) % (2 * Math.PI);
            valO = (valO + Math.random() * 2.0) % (2 * Math.PI);
            valR = (valR + Math.random() * 2.5) % (2 * Math.PI);
            numYellow +=  1.5 * Math.sin(valY);
            numOrange += 2.0 * Math.sin(valO);
            numRed += 2.5 * Math.sin(valR);
        }

        numYellow = Math.min(7, Math.max(3, numYellow));
        numOrange = Math.min(9, Math.max(4, numOrange));
        numRed = Math.min(11, Math.max(5, numRed));

        // total = numRed;
        // difference = total - prevTotalR;
        // prevTotalR = total;
        // if(Math.abs(difference) > 1.5) {
        //     numRed += 1.5 * (difference) / Math.abs(difference);
        // }
        // SmartDashboard.putNumber("change red", (difference) / Math.abs(difference));

        // total = numOrange;
        // difference = total - prevTotalO;
        // prevTotalO = total;
        // if(Math.abs(difference) > 1.5) {
        //     numOrange += 1.5 * (difference) / Math.abs(difference);
        // }
        // SmartDashboard.putNumber("change orange", (difference) / Math.abs(difference));

        // total = numYellow;
        // difference = total - prevTotalY;
        // prevTotalY = total;
        // if(Math.abs(difference) > 1.5) {
        //     numYellow += 1.5 * (difference) / Math.abs(difference);
        // }
        // SmartDashboard.putNumber("change yellow", (difference) / Math.abs(difference));

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