package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Lights extends SubsystemBase {
    
    RobotContainer r;

    PowerDistribution pdh;
    
    AddressableLED leds;
    AddressableLEDBuffer ledBuffer;
 
    public Lights(RobotContainer r){
        this.r = r;
        pdh = new PowerDistribution(21, ModuleType.kRev);

        leds = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(300);
        leds.setLength(ledBuffer.getLength());
        leds.setData(ledBuffer);
        leds.start();
    }


    @Override
    public void periodic(){
        
        if(r.state.hasNote){
            pdh.setSwitchableChannel(true);
        } else {
            pdh.setSwitchableChannel(false);
        }

        rainbow();
        
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

}
