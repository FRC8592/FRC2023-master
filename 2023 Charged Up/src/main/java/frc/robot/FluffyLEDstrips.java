package frc.robot;

import java.awt.Color;

import com.ctre.phoenix.CANifier.PWMChannel;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Parity;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class LEDstrips {
    
    private AddressableLED liftNEOPIXELS;
    private AddressableLEDBuffer liftBuffer;
    private Timer timer;

    private int count = 0;

    final int LED_LENGTH = 45;

    public LEDstrips(){
        liftNEOPIXELS = new AddressableLED(0);
        liftBuffer = new AddressableLEDBuffer(LED_LENGTH);
        liftNEOPIXELS.setLength(LED_LENGTH);
        timer = new Timer();
    }
    
    public void setOrangeBlue(){
        for(int i = 0; i < LED_LENGTH; i++){
            if(i < LED_LENGTH/2){
                liftBuffer.setRGB(i, 243, 50, 0);
            }else{
                liftBuffer.setRGB(i, 0, 80, 133);
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
                        liftBuffer.setRGB(i,243, 50, 0);
                    }
               /*     else if(Math.sin((double)(i + count)) > -.5){
                        liftBuffer.setRGB(i, 0, 255, 0);
                    } */
                     else  {
                        liftBuffer.setRGB(i, 0, 80, 133);
                    }
                }
            }
        liftNEOPIXELS.setData(liftBuffer);
        liftNEOPIXELS.start();
        System.out.println("LED METHOD RUNNING");
        
    }
}