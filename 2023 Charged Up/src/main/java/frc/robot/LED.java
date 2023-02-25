package frc.robot;

import java.awt.Color;

import javax.xml.validation.SchemaFactory;

import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class LED {

    //Hardware objects
    private AddressableLED liftNEOPIXELS;
    private AddressableLEDBuffer liftBuffer;
    private Timer uadTimer = new Timer();
    private Timer visionTimer = new Timer();
    private Timer lowVoltTimer = new Timer();
    private Timer LVPulseTimer = new Timer();
    private Timer snakeTimer = new Timer();
    private Timer blinkTimer = new Timer();
    private Timer lockTimer = new Timer();
    private Vision vision;
    private Power power;

    //Settings
    private LEDMode mode = LEDMode.OFF;
    private double brightnessMultiplier = 1;

    //Dymanic/UpAndDown
    private int uadIndex = 0;
    private PresetColor col1 = PresetColor.OFF;
    private PresetColor col2 = PresetColor.OFF;
    
    //Dynamic/Fire
    private int fireSpawnIDX = 0;
    private ArrayList<Blob> fireBlobs = new ArrayList<Blob>();
    private boolean lowVolts = false;

    //Dynamic/Waves
    private int waveCounter = 0;
    private int indexOn = 0;
    
    //Dynamic/Binary
    private Color[] binary;
    private int binaryIndex;
    
    //Dynamic/Other
    private double timeout;
    
    //Constants
    private static final int LED_LENGTH = 42;
    private static final double MINIMUM_VOLTAGE = 9.0;
    private static final int PULSE_METHOD_SPEED = 4;
    private static final int PULSE_SIZE = 3; 

    /**
     * Premade color presets
     */
    public enum PresetColor {
        RED (255, 0, 0),
        GREEN (0, 255, 0),
        CYAN (0, 160, 255),
        TEAL (0,80,30),
        BLUE (0, 0, 255),
        YELLOW (255, 128, 0),
        PURPLE (138, 0, 255),
        ORANGE (255, 70, 0),
        WHITE (255, 255, 255),
        BROWN (74, 23, 0),
        OFF (0, 0, 0),
        GRAY (80,80,80);
        public final int red;
        public final int blue;
        public final int green;
        PresetColor(int red, int green, int blue){
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
    }

    // public enum LEDPattern {
    //     SOLID,
    //     UP_AND_DOWN,
    //     BINARY,
    //     FIRE,
    //     WAVES,
    //     TARGET_LOCK,
    //     SNAKE,
    //     OFF;
    // }
    public enum LEDMode{
        CUBE,
        CONE,
        TARGETLOCK,
        STOPPLACING,
        ATTENTION,
        SOLID,
        BLINK,
        UP_AND_DOWN,
        BINARY,
        FIRE,
        WAVES,
        TARGET_LOCK,
        SNAKE,
        DEFAULT,
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
        liftBuffer = new AddressableLEDBuffer(LED_LENGTH+1);
        liftNEOPIXELS.setLength(LED_LENGTH+1);

        //Vision and power objects; see above
        this.vision = vision;
        this.power = power;

        Color i = new Color(0,80,30); // 1
        Color o = new Color(32,8,0); // 0
        Color s = new Color(0,0,0); // Separation color
        binary = new Color[] {  s,s,o,i,o,o,i,i,i,o, //N
                                s,s,o,i,i,o,o,i,o,i, //e
                                s,s,o,i,i,i,o,i,i,i, //w
                                s,s,o,i,i,i,o,i,o,o, //t
                                s,s,o,i,i,o,i,i,i,i, //o
                                s,s,o,i,i,o,i,i,i,o, //n
                                s,s,o,i,o,i,i,i,i,o, //^
                                s,s,o,o,i,i,o,o,i,o, //2
                                s,s,o,o,o,o,o,o,o,o};

        LVPulseTimer.start();
        snakeTimer.start();
        blinkTimer.start();
        lockTimer.start();
    }
    /**Update the LEDs
     * @param testLow A boolean for whether to force the LEDs to act as if the battery voltage is low.
     */
    public void updatePeriodic(boolean testLow) {
        //Update the Power object
        power.powerPeriodic();
        switch(mode){
            case CONE:
                setUpAndDown(PresetColor.YELLOW, PresetColor.OFF);
                break;
            case CUBE:
                setUpAndDown(PresetColor.PURPLE, PresetColor.OFF);
                break;
            case TARGETLOCK:
                setTargetLock();
                break;
            case STOPPLACING:
                setBlink(PresetColor.RED, 15);
            case ATTENTION:
                setUpAndDown(PresetColor.CYAN, PresetColor.ORANGE);
            case UP_AND_DOWN:
                setUpAndDown(col1, col2);
                break;
            case FIRE:
                setFire(true);
                break;
            case BINARY:
                setBinary();
                break;
            case WAVES:
                setWaves(col1, col2);
                break;
            case TARGET_LOCK:
                setTargetLock();
                break;
            case SNAKE:
                setSnake(col1);
                break;
            case SOLID:
                setSolid(col1);
                break;
            case OFF:
                setOff();
                break;
        }
        if(mode != LEDMode.FIRE && mode != LEDMode.OFF && fireBlobs.size() > 0){
            fireBlobs=new ArrayList<Blob>();
        }
        //If we have low voltage
        if ((power.voltage < MINIMUM_VOLTAGE || lowVolts || testLow) && power.voltage>0) {
            lowVoltTimer.start();
            lowVolts = true;
            if((int)(LVPulseTimer.get()*3)%2==0){
                setColor(0,PresetColor.RED);
                setColor(1,PresetColor.RED);
                setColor(2,PresetColor.RED);
                setColor(LED_LENGTH,PresetColor.RED);
                setColor(LED_LENGTH-1,PresetColor.RED);
                setColor(LED_LENGTH-2,PresetColor.RED);
            }
            if (lowVoltTimer.get() > 2) {
                lowVoltTimer.stop();
                lowVoltTimer.reset();
                lowVolts = false;
            }
        }
        updateLEDs();
    }


    /*|||||||||||||||||||||||||||||||||||||||||||||||||PATTERN METHODS|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/


    /**
     * Run two colors up and down the LED strip
     * 
     * @param colorA    first color
     * @param colorB    second color
     */
    public void setUpAndDown(PresetColor colorA, PresetColor colorB){
        uadTimer.start();
            if(uadTimer.get() >= .05){
                uadIndex++;
                uadTimer.reset();
                for(int i = 0; i < LED_LENGTH; i++){
                    if (Math.sin(i + uadIndex)  > 0){
                        setColor(i, colorA);
                    }
                     else  {
                        setColor(i, colorB);
                    }
                }
            }
    }
    public void setTargetLock(){
        for(int i = 0; i < LED_LENGTH; i++){
            setColor(i, PresetColor.OFF);
        }
        vision.updateVision();
        if (vision.distanceToTarget() < 80 && vision.distanceToTarget() >= 0.0) {
            visionTimer.start();
            if (visionTimer.get() > 0.5) {
                PresetColor color = PresetColor.RED;
                // Formula to calculate how many LEDs to set in each strip
                // max = max distance before LEDs start lighting up
                // min = distance until piece is "in range"
                double max = 2.0;
                double min = 0.75;
                double difference = max - min;
                int numLEDs = (int)((max - (vision.distanceToTarget() * Constants.INCHES_TO_METERS)) / difference * (LED_LENGTH / 2 - 0.5));
        
                //If distance is at or closer to the max distance, set the color of LEDs to green and cap amount to turn on
                if (numLEDs >= LED_LENGTH / 2) {
                    numLEDs = LED_LENGTH / 2;
                    color = PresetColor.GREEN;
                }
        
                // loop through one side of the LEDs and set an amount of LEDs on depending on distance
                for (int ledIndex = 0; ledIndex < LED_LENGTH / 2; ledIndex++){
                    if(ledIndex < numLEDs) {
                        setColor(ledIndex, color);
                        setColor((LED_LENGTH - 1) - ledIndex, color);
                    } else {
                        setColor(ledIndex, PresetColor.OFF);
                        setColor((LED_LENGTH - 1) - ledIndex, PresetColor.OFF);
                    }
                }
            }
        } else {
            visionTimer.stop();
        }
    }
    public void setFire(boolean runFire) {
        //Random amount for the fireSpawnIDX to more forward
        int random=(int)Math.random();

        //If fireSpawnIDX + random or fireSpawnIDX + 1 results in a multiple of 12, set the fireSpawnIDX to the multiple of 12
        if(fireSpawnIDX+1+random%16!=0&&fireSpawnIDX+1%16==0){
            fireSpawnIDX++;
        }
        else{
            fireSpawnIDX+=1+random;
        }

        //If the fireSpawnIDX number is divisible by 12
        if(fireSpawnIDX%16==0&&runFire){
            //Add a new blob of color. See the private Blob class for more.
            fireBlobs.add(new Blob(0.05+Math.random()*0.15, 
            12+Math.random()*2, 
            (8+Math.random()*5)*0.3,
            0,128,0.1));
        }
        int k = fireBlobs.size();
        //                                              Go through the fireBlobs list,
        for(int i = 0; i < k; i++){
            //                                          update the fireBlobs and check if they're at the top while doing so,
            int increase = fireBlobs.get(i).updateBlob(i);
            if(increase==-1){
                //                                      and remove the blob and modify the variables to account for that if so.
                fireBlobs.remove(i);
                i--;
                k--;
            }
            else{
                k+=increase;
            }
        }

        //Sort the fireBlobs; red first, yellow last. During the double-for loop below, this will mean that yellow is drawn on top.
        Collections.sort(fireBlobs);

        //Clear the LED list
        for(int i = 0; i < LED_LENGTH; i++){
            setColor(i,new Color(0,0,0));
            setColor(LED_LENGTH-i,new Color(0,0,0));
        }

        //                                              For all fireBlobs,
        for(int i = 0; i < fireBlobs.size(); i++){
            //                                          for all indexes in the blob (aka for the area the blob covers)
            for(int j : fireBlobs.get(i).getIndexes()){
                //                                      if the index is actually on the LEDs in real life,
                if(j>-1){
                    //                                  assign the LED at the index to the color of the blob
                    setColor(j, fireBlobs.get(i).getColor());
                    setColor(LED_LENGTH-j, fireBlobs.get(i).getColor());
                }
            }
        }
    }
    public void setBinary(){
        binaryIndex++;
        for(int i = 0; i < 21; i++){
            setColor(20-i,binary[(binaryIndex/6+i)%90]);
            setColor(22+i,binary[(binaryIndex/6+i)%90]);
        }
    }
    public void setWaves(PresetColor col1, PresetColor col2) {
        waveCounter++;
        for(int i = 0; i < LED_LENGTH / 4; i++) {
            if(Math.abs(LED_LENGTH / 4 + i - indexOn) % 5 < PULSE_SIZE) {
                setColor((LED_LENGTH / 4 + i), col1);
                setColor((LED_LENGTH / 4 - 1 - i), col1);
                setColor(LED_LENGTH-(LED_LENGTH / 4 + i), col1);
                setColor(LED_LENGTH-(LED_LENGTH / 4 - 1 - i), col1);
            } else {
                setColor((LED_LENGTH / 4 + i), col2);
                setColor((LED_LENGTH / 4 - 1- i), col2);
                setColor(LED_LENGTH-(LED_LENGTH / 4 + i), col2);
                setColor(LED_LENGTH-(LED_LENGTH / 4 - 1- i), col2);
            }
        }
        if (waveCounter >= PULSE_METHOD_SPEED) {
            waveCounter = 0;
            indexOn = (indexOn + 1) % (LED_LENGTH / 2);
        }
    }
    public void setSnake(PresetColor col1){
        Color color = new Color(col1.red, col1.green, col1.blue);
        int location;
        int direction;
        double speed = 50;
        if(snakeTimer.get()*speed<21){
            location = (int)(snakeTimer.get()*speed);
            direction=0;
        }
        else if (snakeTimer.get()*speed<LED_LENGTH){
            location = 21-(((int)(snakeTimer.get()*speed))-21);
            direction=1;
        }
        else{
            snakeTimer.reset();
            location=1;
            direction=0;
        }
        for(int i = 0; i < LED_LENGTH; i++){
            setColor(i,PresetColor.OFF);
        }
        if(direction==0){
            for(int i = 0; i < 15; i++){
                if(location+i<21){
                    setColor(location+i,getColorAtBrightness(color, 0.0666*i));
                    setColor(21+location+i,getColorAtBrightness(color, 0.0666*i));
                }
                else {
                    setColor(21-((location+i)-21),getColorAtBrightness(color, 0.0666*i));
                    setColor(LED_LENGTH-((location+i)-21),getColorAtBrightness(color, 0.0666*i));
                }
            }
        }
        else{
            for(int i = 0; i < 15; i++){
                if(location-i>0){
                    setColor(location-i,getColorAtBrightness(color, 0.0666*i));
                    setColor(21+location-i,getColorAtBrightness(color, 0.0666*i));
                }
                else{
                    setColor(-(location-i),getColorAtBrightness(color, 0.0666*i));
                    setColor(21-(location-i),getColorAtBrightness(color, 0.0666*i));
                }
            }
        }
    }
    public void setSolid(PresetColor col1){
        for(int i = 0; i < LED_LENGTH; i++){
            setColor(i, col1);
        }
    }
    public void setBlink(PresetColor col1, int speed){
        if(((int)blinkTimer.get()*speed)%2==0){
            setSolid(col1);
        }
        else{
            setSolid(PresetColor.OFF);
        }
    }
    public void setOff(){
        setFire(false);
    }

    /*||||||||||||||||||||||||||||||||||||||||||UTILITY METHODS|||||||||||||||||||||||||||||||||||||||||||||||||||||||*/


    /**
     * Set the LED pattern and colors
     * 
     * @param p {@code LEDPattern} to set
     * @param col1 {@code PresetColor} for the first color. Can be null for binary, fire, and off.
     * @param col2 {@code PresetColor} for the second color. Can be null for binary, fire, and off.
     */
    public void set(LEDMode m, PresetColor col1, PresetColor col2) {
        if(lockTimer.get()>timeout){
            this.mode = m;
            this.col1 = col1;
            this.col2 = col2;
            if(m==LEDMode.CONE){
                setLockTimer(2.0);
            }
        }
    }
    public void set(LEDMode m){
        set(m, PresetColor.OFF, PresetColor.OFF);
    }
    /**
     * Sets the brightness level for LEDS
     * 
     * @param pct percent of brightness (0 - 100)
     */
    public void setBrightness(double pct) {
        brightnessMultiplier = pct / 100.0;
    }

    public Color getColorAtBrightness(Color c, double b){
        return new Color((int)(c.getRed()*b),(int)(c.getGreen()*b),(int)(c.getBlue()*b));
    }
    /**
     * Set the current leds to a certain color
     * 
     * @param i         index of the led light to set
     * @param color     PresetColor to set the light
     */
    private void setColor(int i, PresetColor color){
        liftBuffer.setRGB(i, (int)(color.red * brightnessMultiplier), (int)(color.green * brightnessMultiplier), (int)(color.blue * brightnessMultiplier));
    }
    /**
     * Set the current leds to a certain color
     * 
     * @param i         index of the led light to set
     * @param color     Color to set the light
     */
    private void setColor(int i, Color color){
        liftBuffer.setRGB(i, (int)(color.getRed() * brightnessMultiplier), (int)(color.getGreen() * brightnessMultiplier), (int)(color.getBlue() * brightnessMultiplier));
    }
    private void updateLEDs(){
        liftNEOPIXELS.setData(liftBuffer);
        liftNEOPIXELS.start();
    }
    private void setLockTimer(double timeout){
        lockTimer.reset();
        lockTimer.start();
        this.timeout=timeout;
    }

    /*||||||||||||||||||||||||||||||||||||||||||||||||||||||PRIVATE CLASS|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||*/


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
         * @param location The location of the blob
         * @param green The green value of the color of this blob
         * @param heightChange The speed, in LEDs/frame, that this blob shrinks
         */
        public Blob(double speed, double height, double colorChange, double location, double green, double heightChange){
            this.speed = speed;
            this.location = location;
            this.height = height;
            this.green = green;
            this.colorChange = colorChange;
            this.heightChange=heightChange;
        }
        public int updateBlob(int index){
            //If we're at the top, decrease the height (the height determines the bottom of the blob, not the top)
            if(location>21){
                height-=speed;
            }
            //Otherwise if the blob is low or small, move up
            else if (location < 8||height<7){
                location+=speed;
            }
            //If the blob is high and large, split into smaller blobs
            else{
                int half = (int)(height/2);
                fireBlobs.remove(index);
                fireBlobs.add(new Blob(speed, half, colorChange, location, green, heightChange/1.5));
                fireBlobs.add(new Blob(speed, height-half, colorChange, location-half, green, heightChange/2));
                return 1;
            }
            //Increase the movement speed, the height change speed, and the height
            speed+=0.01;
            heightChange+=heightChange/32;
            height-=heightChange;

            //If the height is negative, return false (above, doing this deletes this blob)
            if(height<0){
                return -1;
            }

            //Redshift the blob
            green-=colorChange;
            if(green<0){
                green=0;
            }
            return 0;
        }

        //Self-explanatory methods
        public double getGreen(){
            return green;
        }
        public int compareTo(Blob b){
            return (int)(this.green - b.getGreen());
        }
        public Color getColor(){
            return new Color(255, (int)green,0);
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
}