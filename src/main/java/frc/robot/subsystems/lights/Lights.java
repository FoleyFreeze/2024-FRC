package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Lights extends SubsystemBase {
    
    RobotContainer r;

    PowerDistribution pdh;
    
    AddressableLED leds;
    AddressableLEDBuffer ledBuffer;

    Timer timer;
 
    public Lights(RobotContainer r){
        this.r = r;
        pdh = new PowerDistribution(21, ModuleType.kRev);

        leds = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(210);
        leds.setLength(ledBuffer.getLength());
        leds.setData(ledBuffer);
        leds.start();

        timer = new Timer();
        timer.start();
    }


    @Override
    public void periodic(){
        
        if(r.state.hasNote){
            pdh.setSwitchableChannel(true);
        } else {
            pdh.setSwitchableChannel(false);
        }

        rainbow();
        //testPattern();
        
    }

    private int firstPixel = 0;
    private void rainbow(){
        for(int i=0;i<ledBuffer.getLength();i++){
            int hue = (firstPixel + (i*180 / ledBuffer.getLength())) % 180;
            ledBuffer.setHSV(i,hue,255,120);
        }

        firstPixel += 3;
        firstPixel %= 180;
        leds.setData(ledBuffer);
    }

    private void testPattern(){
        if(timer.get() > 1){
            timer.reset();

            int i=0;
            int idx;
            for(;i<50;i++){
                int level = 255 - i;
                idx = (i+firstPixel) % 300;
                ledBuffer.setRGB(idx,level,0,0);
            }
            for(;i<100;i++){
                int level = 255 - i + 50;
                idx = (i+firstPixel) % 300;
                ledBuffer.setRGB(idx,0,level,0);
            }
            for(;i<150;i++){
                int level = 255 - i + 100;
                idx = (i+firstPixel) % 300;
                ledBuffer.setRGB(idx,0,0,level);
            }
            for(;i<300;i++){
                idx = (i+firstPixel) % 300;
                ledBuffer.setRGB(idx,0,0,0);
            }
            leds.setData(ledBuffer);
            
            firstPixel++;
            if(firstPixel >= 300) firstPixel = 0;
        }
    }

}
