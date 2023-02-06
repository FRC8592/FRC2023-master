package frc.robot;

import frc.robot.Color;



import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Parity;
import edu.wpi.first.wpilibj.SerialPort.Port;


public class LED {
    
    private AddressableLED liftNEOPIXELS;
    private AddressableLEDBuffer liftBuffer;
    private Timer timer;

    private int count = 0;

    final int LED_LENGTH = 45;

    public void setColor(int i, Color color){
        liftBuffer.setRGB(i, color.red, color.green, color.blue);
    }
    public LED(){
        liftNEOPIXELS = new AddressableLED(0);
        liftBuffer = new AddressableLEDBuffer(LED_LENGTH);
        liftNEOPIXELS.setLength(LED_LENGTH);
        timer = new Timer();
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


    public void upAndDown(){
        timer.start();

       
            if(timer.get() >= .05){
                
                count++;
                timer.reset();
                if(count > LED_LENGTH){
                    count = 0;
                }
                for(int i = 0; i < LED_LENGTH; i++){
                    if (Math.sin(1 * (double)(i + count))  > 0){
                        setColor(i, Color.ORANGE);
                    }
               /*     else if(Math.sin((double)(i + count)) > -.5){
                        liftBuffer.setRGB(i, 0, 255, 0);
                    } */
                     else  {
                        setColor(i, Color.BLUE);
                    }
                }
            }
        liftNEOPIXELS.setData(liftBuffer);
        liftNEOPIXELS.start();
        System.out.println("LED METHOD RUNNING");
        
    }

    public void setFull(Color color){
        for (int ledIndex = 0; ledIndex < LED_LENGTH; ledIndex++){
            setColor(ledIndex, color);
        }
        liftNEOPIXELS.setData(liftBuffer);
        liftNEOPIXELS.start();
        System.out.println("LED METHOD RUNNING");   
        
    }
    
    public void setHalf(Color color){
        for (int ledIndex = 0; ledIndex < LED_LENGTH; ledIndex++){
            if (ledIndex % 2 == 0){
                setColor(ledIndex, color);
            }else {
                setColor(ledIndex, Color.OFF);
            }
        }
        liftNEOPIXELS.setData(liftBuffer);
        liftNEOPIXELS.start();
        System.out.println("LED METHOD RUNNING");   
    }

    public void setOff(){
        for (int ledIndex = 0; ledIndex < LED_LENGTH; ledIndex++){
            setColor(ledIndex, Color.OFF);
        }
        liftNEOPIXELS.setData(liftBuffer);
        liftNEOPIXELS.start();
        System.out.println("LED METHOD RUNNING");   
    }

    /**
     * Set more LEDs on depending on how close you are to a target
     * 2m = max distance
     * 0.5m = closest distance
     * 
     * @param distToTarget  the distance to the target (meters)
     */
    public void setProximity(double distToTarget) {
        int numLEDs = (int)((2 - distToTarget) / 1.5 * LED_LENGTH);
        Color color = Color.RED;
        if(numLEDs > LED_LENGTH) {
            numLEDs = LED_LENGTH;
            color = Color.GREEN;
        }

        for (int ledIndex = 0; ledIndex < numLEDs; ledIndex++){
            setColor(ledIndex, color);
        }
        liftNEOPIXELS.setData(liftBuffer);
        liftNEOPIXELS.start();
        System.out.println("LED METHOD RUNNING"); 
    }
}