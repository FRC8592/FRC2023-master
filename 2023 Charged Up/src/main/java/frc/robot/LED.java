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
        OFF (0, 0, 0),
        GRAY (80,80,80);
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
    /**Construct an LED controller
     * 
     * @param power Power object to get battery voltage
     * @param vision Vision object for the proximity method
     */
    public LED(Power power, Vision vision){
        //Get the physical LEDs ready
        liftNEOPIXELS = new AddressableLED(0);
        liftBuffer = new AddressableLEDBuffer(LED_LENGTH);
        liftNEOPIXELS.setLength(LED_LENGTH);
        
        //Timer to make the UpAndDown method work
        timer = new Timer();
        
        //Vision and power objects; see above
        this.vision = vision;
        this.power = power;
        
        //UpAndDown colors
        col1=Color.OFF;
        col2=Color.OFF;
        
        //ArrayList for the fire method
        blobs=new ArrayList<Blob>();
        
        //Array to store the LEDs, also for the fire method
        LEDs=new SixteenMColor[42];
        for(int i = 0; i < 42; i++){
            LEDs[i]=new SixteenMColor(0,0,0);
        }
    }
    /**Update the LEDs
     * @param testLow A boolean for whether to force the LEDs to act as if the battery voltage is low.
     */
    public void updatePeriodic(boolean testLow) {
        //Update the Power object
        power.powerPeriodic();
        
        //If we have low voltage
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
        //If we want to do the proximity-targetlock-thing
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
        //Temporarily, attention mode runs the fire method
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
                //Need to revise this
                case ATTENTION:
                    col1=Color.CYAN;
                    break;
                case OFF:
                    col1=Color.OFF;
                    blobs=new ArrayList<Blob>();
                    break;
            }
            //Run the UpAndDown with the two colors
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
    /**
     * Set the current leds to a certain color
     * 
     * @param i         index of the led light to set
     * @param color     SixteenMColor to set the light
     */
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
    private void setFire() {
        //Random amount for the frame to more forward
        int random=(int)Math.random()*5;

        //If frame + random or frame + 1 results in a multiple of 12, set the frame to the multiple of 12
        if(frame+1+random%16!=0&&frame+1%16==0){
            frame++;
        }
        else{
            frame+=1+random;
        }
        
        //If the frame number is divisible by 12
        if(frame%16==0){
            //Add a new blob of color. See the private Blob class for more.
            blobs.add(new Blob(0.05+Math.random()*0.15, 12+Math.random()*2, (8+Math.random()*5)*0.3));
        }
        int k = blobs.size();
        //                                              Go through the blobs list,
        for(int i = 0; i < k; i++){
            //                                          update the blobs and check if they're at the top while doing so,
            if(!blobs.get(i).updateBlob()){
                //                                      and remove the blob and modify the variables to account for that if so.
                blobs.remove(i);
                i--;
                k--;
            }
        }
        
        //Sort the blobs; red first, yellow last. During the double-for loop below, this will mean that yellow is drawn on top.
        Collections.sort(blobs);
        
        //Clear the LED list
        for(int i = 0; i < LEDs.length; i++){
            LEDs[i]=new SixteenMColor(0,0,0);
        }
        
        //                                              For all blobs,
        for(int i = 0; i < blobs.size(); i++){
            //                                          for all indexes in the blob (aka for the area the blob covers)
            for(int j : blobs.get(i).getIndexes()){
                //                                      if the index is actually on the LEDs in real life,
                if(j>-1){
                    //                                  assign the LED at the index to the color of the blob
                    LEDs[j]=blobs.get(i).getColor();
                }
            }
        }
        
        //Now that we're done figuring out what has what color, set the LEDs
        for(int i = 0; i < LEDs.length/2; i++){
            setColor(i, LEDs[i]);
            setColor(42-i, LEDs[i]);
        }
        liftNEOPIXELS.setData(liftBuffer);
        liftNEOPIXELS.start();
    }
    
    //Blob of color in the fire
    private class Blob implements Comparable<Blob> {
        private double speed;
        private double location;
        private double height;
        private double heightChange;
        private double colorChange;
        private double red;
        private double green;
        
        /**Create a blob of color
         * @param speed The starting speed of the blob
         * @param height The height of the blob
         * @param colorChange The speed that the blob changes from yellow to red
         */
        public Blob(double speed, double height, double colorChange){
            this.speed = speed;
            this.location = 0;
            this.height = height;
            this.red = 255;
            this.green = 128;
            this.colorChange = colorChange;
            this.heightChange=0.1;
        }
        public boolean updateBlob(){
            //If we're at the top, decrease the height (the height determines the bottom of the blob, not the top)
            if(location>21){
                height-=speed;
            }
            //Otherwise move up
            else{
                location+=speed;
            }
            //Increase the movement speed, the height change speed, and the height
            speed+=0.01;
            heightChange+=heightChange/32;
            height-=heightChange;
            
            //If the height is negative, return false (above, doing this deletes this blob)
            if(height<0){
                return false;
            }
            
            //Redshift the blob
            green-=colorChange;
            if(green<0){
                green=0;
            }
            return true;
        }
        
        //Couple self-explanatory methods
        public double getGreen(){
            return green;
        }
        public int compareTo(Blob b){
            return (int)(this.green - b.getGreen());
        }
        public SixteenMColor getColor(){
            return new SixteenMColor((int)red, (int)green,0);
        }
        
        //Get a list of indexes that this blob covers
        public int[] getIndexes(){
            int intHeight = (int)height;
            if(intHeight<0){
                return new int[0];
            }
            int[] result = new int[intHeight];
            for(int i = 0; i < intHeight; i++){
                result[i]=(int)location-i;
            }
            return result;
        }
    }
    
    //Dubplicate of the built-in Color class.
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