package frc.robot;

import java.awt.Color;

import javax.xml.validation.SchemaFactory;

import java.util.ArrayList;
import java.util.Collections;

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
    private Power power;
    private Color col1;
    private Color col2;
    private double heightChange;
    private double heightChangeSineChange;
    private SixteenMColor[] LEDs;
    private int frame;
    ArrayList<Blob> blobs;
    final int LED_LENGTH = 43;

    /**
     * Premade color presets
     */
    public enum Color {
        RED (255, 0, 0),
        GREEN (0, 255, 0),
        CYAN (0, 160, 255),
        BLUE (0, 0, 255),
        YELLOW (255, 128, 0),
        PURPLE (138, 0, 255),
        ORANGE (255, 100, 0),
        WHITE (255, 255, 255),
        BROWN (74, 23, 0),
        DARKRED (50,0,0),
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

    public LED(Power power, Vision vision){
        liftNEOPIXELS = new AddressableLED(0);
        liftBuffer = new AddressableLEDBuffer(LED_LENGTH);
        liftNEOPIXELS.setLength(LED_LENGTH);
        timer = new Timer();
        blinkSpeedTimer = new Timer();
        this.vision = vision;
        this.power = power;
        col1=Color.OFF;
        col2=Color.OFF;
        blobs=new ArrayList<Blob>();
        LEDs=new SixteenMColor[42];
        for(int i = 0; i < 42; i++){
            LEDs[i]=new SixteenMColor(0,0,0);
        }
    }

    public void updatePeriodic(boolean testLow) {
        power.powerPeriodic();
        SmartDashboard.putString("LED Mode", mode.name());
        SmartDashboard.putBoolean("LowVoltage", testLow);
        if (power.voltage < 9.0 || lowVolts || testLow) {
            delayTimer.start();
            lowVolts = true;
            col2=Color.DARKRED;
            if (delayTimer.get() > 2) {
                delayTimer.stop();
                delayTimer.reset();
                lowVolts = false;
                col2=Color.OFF;
            }
        }
        if(mode==LEDMode.TARGETLOCK){
            vision.updateVision();
            if (vision.distanceToTarget() < 80 && vision.distanceToTarget() >= 0.0) {
                delayTimer.start();
                if (delayTimer.get() > 0.5) {
                    setProximity(vision.distanceToTarget() * Constants.INCHES_TO_METERS);
                }
            } else {
                delayTimer.stop();
            }
        }
        else if(mode==LEDMode.ATTENTION){
            setFire();
        }
        else{
            switch(mode){
                case CONE:
                    col1=Color.YELLOW;
                    break;
                case CUBE:
                    col1=Color.PURPLE;
                    break;
                case STOPPLACING:
                    col1=Color.WHITE;
                    break;
                case ATTENTION:
                    col1=Color.CYAN;
                    break;
                case OFF:
                    col1=Color.OFF;
                    blobs=new ArrayList<Blob>();
                    break;
            }
            upAndDown(col1, col2);
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
    public void setColor(int i, Color color){
        liftBuffer.setRGB(i, (int)(color.red * brightnessMultiplier), (int)(color.green * brightnessMultiplier), (int)(color.blue * brightnessMultiplier));   
    }
    public void setColor(int i, SixteenMColor color){
        liftBuffer.setRGB(i, (int)(color.getRed() * brightnessMultiplier), (int)(color.getGreen() * brightnessMultiplier), (int)(color.getBlue() * brightnessMultiplier));   
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
        setPulseState(BlinkSpeed.NORMAL);
        setPct(100, Color.RED);
    }

    private void setFire() {
        int random=(int)Math.random()*5;
        if(frame+1+random%12!=0&&frame+1%12==0){
            frame++;
        }
        else{
            frame+=1+random;
        }
        if(frame%12==0){
            blobs.add(new Blob(0.2, 6+Math.random()*8, 3+Math.random()*15));
        }
        int k = 0;
        for(int i = 0; i < blobs.size(); i++){
            if(!blobs.get(i).updateBlob()){
                blobs.remove(k);
                i++;
            }
        }
        Collections.sort(blobs);
        for(int i = 0; i < LEDs.length; i++){
            LEDs[i]=new SixteenMColor(0,0,0);
        }
        for(int i = 0; i < blobs.size(); i++){
            for(int j : blobs.get(i).getIndexes()){
                if(j>-1){
                    LEDs[j]=blobs.get(i).getColor();
                }
            }
        }
        for(int i = 0; i < LEDs.length/2; i++){
            setColor(i, LEDs[i]);
            setColor(42-i, LEDs[i]);
        }
        liftNEOPIXELS.setData(liftBuffer);
        liftNEOPIXELS.start();
    }
    private class Blob implements Comparable<Blob> {
        private double speed;
        private double location;
        private double height;
        private double heightChange;
        private double colorChange;
        private double red;
        private double green;
        public Blob(double speed, double height, double colorChange){
            this.speed = speed;
            this.location = 0;
            this.height = height;
            this.red = 255;
            this.green = 255;
            this.colorChange = colorChange;
            this.heightChange=0.1;
        }
        public boolean updateBlob(){
            if(location>21){
                height-=speed;
            }
            else{
                location+=speed;
            }
            speed+=0.02;
            heightChange+=heightChange/20;
            height-=heightChange;
            if(height<0){
                return false;
            }
            green-=colorChange;
            if(green<0){
                green=0;
            }
            return true;
        }
        public double getGreen(){
            return green;
        }
        public double random(double start, double end){
            return start+(Math.random()*end-start);
        }
        public int compareTo(Blob b){
            return (int)(this.green - b.getGreen());
        }
        public SixteenMColor getColor(){
            return new SixteenMColor((int)red, (int)green,0);
        }
        public int[] getIndexes(){
            int intHeight = (int)height;
            if(intHeight<0){
                System.out.println(heightChange);
                System.out.println(height);
                System.out.println(speed);
                System.out.println(location);
                return new int[0];
            }
            int[] result = new int[intHeight];
            for(int i = 0; i < intHeight; i++){
                result[i]=(int)location-i;
            }
            return result;
        }
    }
    private class SixteenMColor{
        int red;
        int green;
        int blue;
        public SixteenMColor(int red, int green, int blue){
            this.red=red;
            this.green=green;
            this.blue=blue;
        }
        public int getRed(){
            return red;
        }
        public int getGreen(){
            return green;
        }
        public int getBlue(){
            return blue;
        }
    }
}