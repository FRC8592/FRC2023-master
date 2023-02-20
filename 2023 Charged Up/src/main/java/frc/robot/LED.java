package frc.robot;

import javax.xml.validation.SchemaFactory;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* 
 * Reworked class for LEDs, state will be set in robot with setState() method, blink speed with the setBlinkState method, brightness with the setBrightness method,
 * and updatePeriodic() will need to be called in robotPeriodic() to update state
 * 
 * -Cole
 * -main
 */

public class LED {
    
    private AddressableLED liftNEOPIXELS;
    private AddressableLEDBuffer liftBuffer;
    private Timer timer;
    private Timer blinkSpeedTimer;
    private BlinkSpeed blinkSpeed = BlinkSpeed.SOLID;
    private LEDMode mode = LEDMode.OFF;
    private Vision vision = new Vision(Constants.LIMELIGHT_BALL, Constants.BALL_LOCK_ERROR,
    Constants.BALL_CLOSE_ERROR, Constants.BALL_CAMERA_HEIGHT, Constants.BALL_CAMERA_ANGLE, 
    Constants.BALL_TARGET_HEIGHT, Constants.BALL_ROTATE_KP, Constants.BALL_ROTATE_KI, Constants.BALL_ROTATE_KD, new FRCLogger(true, "CustomLogs"));

    private int count = 0;
    private double brightnessMultiplier = 1;
    private boolean lowVolts = false;
    private Timer delayTimer = new Timer();
    private Power power = new Power();
    private int indexOn = 0;
    private int waveCounter = 0;

    final int LED_LENGTH = 21;

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

    /**
     * LED mode presets
     */
    public enum LEDMode {
        // Signal for a cone
        CONE,
        // Signal for a cube
        CUBE,
        // Target lock on a game piece
        TARGETLOCK,
        // Signal to Human Player to stop placing game pieces
        STOPPLACING,
        // Get the attention of the Human Player
        ATTENTION,
        // Waves test
        WAVES,
        // LEDs off
        OFF;
    }

    public LED(){
        liftNEOPIXELS = new AddressableLED(0);
        liftBuffer = new AddressableLEDBuffer(LED_LENGTH);
        liftNEOPIXELS.setLength(LED_LENGTH);
        timer = new Timer();
        blinkSpeedTimer = new Timer();
    }

    /**
     * Periodically update the LEDs based on the current state
     */
    public void updatePeriodic() {
        power.powerPeriodic();
        SmartDashboard.putString("LED Mode", mode.name());
        
        // Switch between the possible states of the LED
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
            // //5m = 196.85
                blinkSpeed = BlinkSpeed.SOLID;
                vision.updateVision();
                if (vision.distanceToTarget() < 80 && vision.distanceToTarget() >= 0.0) {
                    delayTimer.start();
                    if (delayTimer.get() > 0.5) {
                        setProximity(vision.distanceToTarget() * Constants.INCHES_TO_METERS);
                    }
                } else {
                    setPct(50, Color.ORANGE);
                    delayTimer.reset();
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
            case WAVES:
                blinkSpeed = BlinkSpeed.SOLID;
                setWaves(Color.BLUE);
                break;
            case OFF:
                blinkSpeed = BlinkSpeed.SOLID;
                turnOff();
                break;
        }
        
        //power var
        if ((power.voltage < 9.0 || lowVolts)) {
            delayTimer.start();
            lowVolts = true;
            lowVoltage();
            if (delayTimer.get() > 5) {
                delayTimer.reset();
                delayTimer.stop();
                lowVolts = false;
            } 
        }
        
        liftNEOPIXELS.setData(liftBuffer);
        liftNEOPIXELS.start(); 
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
    public void setBlinkState(BlinkSpeed speed) {
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
        int numLEDs = (int)((max - distToTarget) / difference * (LED_LENGTH));
        
        //If distance is at or closer to the max distance, set the color of LEDs to green and cap amount to turn on
        if (numLEDs >= LED_LENGTH ) {
            numLEDs = LED_LENGTH;
            color = Color.GREEN;
            timer.start();
        }

        // loop through one side of the LEDs and set an amount of LEDs on depending on distance
        for (int ledIndex = 0; ledIndex < LED_LENGTH; ledIndex++){
            if(ledIndex < numLEDs) {
                setColor(ledIndex, color);
            } else {
                setColor(ledIndex, Color.OFF);
            }
        }
    }

    /**
     * Check the voltage of the robot, and if lower than a certain value for a certain time, blink a slow red
     */
    // private double voltages[] = new double[10];
    // int indexOn = 0, sum = 0, avg;
    private void lowVoltage() {
        for(int i = 0; i < LED_LENGTH / 10; i++) {
            setColor(i, Color.RED);
        }
        
    }

    private void setWaves(Color color) {
        waveCounter++;
        for(int i = 0; i < LED_LENGTH / 2; i++) {
            if(Math.abs(LED_LENGTH / 2 + i - indexOn) % 5 < Constants.PULSE_SIZE) {
                setColor((LED_LENGTH / 2 + i), color);
                setColor((LED_LENGTH / 2 - 1 - i), color);
            } else {
                setColor((LED_LENGTH / 2 + i), Color.OFF);
                setColor((LED_LENGTH / 2 - 1- i), Color.OFF);
            }
        }
        if (waveCounter >= Constants.PULSE_METHOD_SPEED) {
            waveCounter = 0;
            indexOn = (indexOn + 1) % (LED_LENGTH / 2);
        }
    }
}