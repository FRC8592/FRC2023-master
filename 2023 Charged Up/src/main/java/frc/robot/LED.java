package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Parity;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.Math.*;


public class LED {

    public enum Color {
        RED (128, 0, 0),
        GREEN (0, 128, 0),
        BLUE (0, 80, 133),
        YELLOW (255, 255, 0),
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
    
    private AddressableLED liftNEOPIXELS;
    private AddressableLEDBuffer liftBuffer;
    private Timer timer;

    private int count = 0;

    final int LED_LENGTH = 43;

    public void setColor(int i, Color color){
        liftBuffer.setRGB(i, color.red, color.green, color.blue);
    }
    public LED(){
        liftNEOPIXELS = new AddressableLED(0);
        liftBuffer = new AddressableLEDBuffer(LED_LENGTH);
        liftNEOPIXELS.setLength(LED_LENGTH);
        timer = new Timer();
        SmartDashboard.putBoolean("Proximity On", false);
        SmartDashboard.putBoolean("Pct On", false);
    }

    public void setOrangeBlue(){
        for(int i = 0; i < LED_LENGTH; i++){
            if(i < LED_LENGTH/2){
                setColor(i, Color.ORANGE);
            }else{
                setColor(i, Color.BLUE);
            }
        }
        liftNEOPIXELS.setData(liftBuffer);
        liftNEOPIXELS.start();
        System.out.println("LED METHOD RUNNING");
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
        System.out.println("LED METHOD RUNNING");
        
    }

    // public void setFull(Color color){
    //     for (int ledIndex = 0; ledIndex < LED_LENGTH; ledIndex++){
    //         setColor(ledIndex, color);
    //     }
    //     liftNEOPIXELS.setData(liftBuffer);
    //     liftNEOPIXELS.start();
    //     System.out.println("LED METHOD RUNNING");   
        
    // }
    
    // public void setHalf(Color color){
    //     for (int ledIndex = 0; ledIndex < LED_LENGTH; ledIndex++){
    //         if (ledIndex % 2 == 0){
    //             setColor(ledIndex, color);
    //         }else {
    //             setColor(ledIndex, Color.OFF);
    //         }
    //     }
    //     liftNEOPIXELS.setData(liftBuffer);
    //     liftNEOPIXELS.start();
    //     System.out.println("LED METHOD RUNNING");   
    // }

    // public void setOff(){
    //     for (int ledIndex = 0; ledIndex < LED_LENGTH; ledIndex++){
    //         setColor(ledIndex, Color.OFF);
    //     }
    //     liftNEOPIXELS.setData(liftBuffer);
    //     liftNEOPIXELS.start();
    //     System.out.println("LED METHOD RUNNING");   
    // }

    /**
     * Set a percentage of the LEDs on, automatically spaces out lights
     * 
     * @param pct   the percentage of LEDs to turn on (0 - 100)
     * @param color the color of the LEDs that are on 
     */
    public void setPct(double pct, Color color) {
        SmartDashboard.putBoolean("Pct On", true);
        SmartDashboard.putNumber("Pct LEDs on", pct);
        SmartDashboard.putString("Pct LED Color", color.name());

        for (int ledIndex = 0; ledIndex < LED_LENGTH; ledIndex++){
            if (pct != 0 && (double)ledIndex % (1.0 / (pct / 100.0)) < 1.0){
                setColor(ledIndex, color);
            }else {
                setColor(ledIndex, Color.OFF);
            }
        }
        liftNEOPIXELS.setData(liftBuffer);
        liftNEOPIXELS.start();
        System.out.println("LED METHOD RUNNING");  
    }

    public void turnOff() {
        setPct(100, Color.OFF);
    }

    /**
     * Set more LEDs on depending on how close you are to a target
     * 5m = max distance
     * 0.6m = closest distance
     * 
     * @param distToTarget  the distance to the target (meters)
     */
    public void setProximity(double distToTarget) {
        SmartDashboard.putBoolean("Proximity On", true);
        int numLEDs = (int)((5 - distToTarget) / 4.25 * (LED_LENGTH / 2 - 0.5));
        Color color = Color.RED;
        if(numLEDs >= LED_LENGTH / 2) {
            numLEDs = LED_LENGTH / 2;
            color = Color.GREEN;
        }

        for (int ledIndex = 0; ledIndex < LED_LENGTH / 2; ledIndex++){
            if(ledIndex < numLEDs) {
                setColor(ledIndex, color);
                setColor((LED_LENGTH - 1) - ledIndex, color);
            } else {
                setColor(ledIndex, Color.OFF);
                setColor(LED_LENGTH - 1 - ledIndex, Color.OFF);
            }
        }
        liftNEOPIXELS.setData(liftBuffer);
        liftNEOPIXELS.start();
        System.out.println("LED METHOD RUNNING"); 
    }

    // WIP method
    
    // private boolean first = true;
    // /**
    //  * Create a firelike effect on the LED strips
    //  * 
    //  * @param clrA bottom color of the "fire"
    //  * @param clrB middle color of the "fire"
    //  * @param clrC top color of the "fire"
    //  */
    // public void setFireToTheRobot(Color clrA, Color clrB, Color clrC) {
    //     if(first) {
    //         int numA = (int)(Math.random() * 12 + 1);
    //         int numB = (int)(numA + Math.random() * 8 - Math.random() * 8);
    //         int numC = (int)(numB + Math.random() * 5 - Math.random() * 5);
    //     } else {
            
    //     }
    // }
}