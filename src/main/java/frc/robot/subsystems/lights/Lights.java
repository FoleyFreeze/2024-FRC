package frc.robot.subsystems.lights;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RoboState.ClimbState;


public class Lights extends SubsystemBase{

    public final boolean disabled = false;
    
    RobotContainer r;
    //InputCal cal;

    AddressableLED led;

    AddressableLEDBuffer ledBuffer;
    public PowerDistribution pdh;
    boolean switchableChannel = true;

    Color[] rainbow = {
        new Color(255,0,0),
        new Color(255,127,0),
        new Color(255,255,0),
        new Color(0,255,0),
        new Color(0,0,255),
        new Color(75,0,130),
        new Color(148,0,211)
    };

    //has note
    Color[] garfield = {
        //new Color(245, 90, 200),
        //new Color(215, 100, 175),
        //new Color(239, 96, 190),
        //new Color(209, 85, 178),
        new Color(10, 255, 0),
        new Color(7, 255, 0),
        new Color(12, 255, 0),
        new Color(10, 255, 0),
    };

    //auton
    Color[] brain = {
        new Color(83, 240, 0),
        new Color(75, 246, 0),
        new Color(90, 228, 0),
        new Color(75, 252, 0),
        //new Color (0, 255, 0)
    };

    //has transfer
    Color [] blizzard = {
        new Color(14, 125, 252),
        new Color(12, 111, 223),
        new Color(95, 160, 235),
        new Color(8, 162, 239),        
        new Color(240, 166, 250),
        new Color(10, 140, 205),
        new Color(56, 149, 196),
        new Color(107, 161, 188),
    };

    //climb state DEPLOYED
    Color[] oscar = {
        new Color(47, 245, 16),
        new Color(74, 242, 48),
        new Color(99, 236, 77),
        new Color(100, 228, 92),
        new Color(107, 233, 122),
        new Color(110, 233, 120),
        new Color(100, 238, 100)
    };


    //climb state HOOKED
    Color [] stitch = {
        new Color(185, 238, 234),
        new Color(7, 247, 231),
        new Color(6, 111, 104),
    };

    //climb state CLIMBED
    Color [] snowballFight = {
        new Color(8, 24, 250),
        new Color(8, 170, 250),
        new Color(8, 250, 234),
        new Color(91, 116, 243),
        new Color(7, 221, 245),
    };

    //climb state stageLeft
    Color[] belle = {
        new Color(255, 255, 10),
        new Color(246, 246, 64),
        new Color(247, 172, 21),
        new Color(235, 235, 64),
    };

    //climb state stageCenter
    Color[] beast = {
        new Color(7, 48, 253),
        new Color(7, 230, 255),
        new Color(7, 40, 255),
        new Color(9, 26, 141),
        new Color(9, 141, 132),
    };

    //climb state stageRight
    Color[] gaston = {
        new Color(253, 6, 22),
        new Color(243, 91, 101),
        new Color(239, 169, 173),
        new Color(245, 10, 25),
        new Color(245, 10, 135),
        new Color(229, 88, 163),
        new Color(246, 14, 99),
        new Color(245, 105, 156),
        new Color(253, 6, 22),
    };
    public Lights(RobotContainer r){
        this.r = r;
        //this.cal = cal;
        if(disabled) return;

        initOutputLed();
        ledOutEnable(true);

        led = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(205);

        led.setLength(ledBuffer.getLength());
        led.start();

        pdh = new PowerDistribution(21, ModuleType.kRev);
    }

    public void underglow(boolean on){
        if(on != switchableChannel){
            pdh.setSwitchableChannel(on);
            switchableChannel = on;
        }
    }

    int offset = 0;
    double switchTime = 0;
    @Override
    public void periodic(){
        if(disabled) return;

        SmartDashboard.putNumber("PDH Voltage", pdh.getVoltage());

        //dont underglow when disabled in the pits
        underglow((DriverStation.isFMSAttached() && DriverStation.isDisabled() || r.state.hasNote) /*&& r.inputs.getFieldMode()*/);
        
        if(DriverStation.isDisabled()){
            //disabled
            /*if(r.inputs.selectedLevel == Level.NONE){
                if(DriverStation.getAlliance() == Alliance.Blue){
                    skittles2(blueStripes, false);
                } else {
                    skittles2(redStripes, false);
                }
            } else{
                testMode();
            }*/

            testMode();
            //skittles2(rainbow, true);

            //put in order of important DONT FORGET LIBBY
        } else if(DriverStation.isAutonomous()){
            //auton
            skittles2(brain, true);

        } else if(r.state.climbDeploy == ClimbState.DEPLOYED){
            //DEPLOYED
            skittles2(oscar, false);

        } else if(r.state.climbDeploy == ClimbState.HOOKED){
            //HOOKED
            skittles2(stitch, true);

        } else if(r.state.climbDeploy == ClimbState.CLIMBED){
            //CLIMBED
            skittles2(snowballFight, false);

        } else if(r.state.hasTransfer){
            //has transfer
            skittles2(blizzard, true);

        } else if(r.state.hasNote){
            //has note
            skittles2(garfield, false);
            
        } else { //no note
            skittles2(beast, false);
        }
        
        //is this even possible lol
        /*if(/*stage left switched*){
            //stage Left
            skittles2(belle, true);

        } else if(/*stage right switched*){
            //stage right
            skittles2(gaston, true);

        }else{
            //stage center
            skittles2(beast, false);
        }

        if(led != null){
            led.setData(ledBuffer);
        }*/
    }
    
    public int testStage = 0;
    public void incTestStage(){
        System.out.println("inc test stage: " + testStage);
        testStage++;
    }

    public void testMode(){
        int maxStage = 9;
        if(testStage > maxStage) testStage = 0;

        switch(testStage){
            case 0:
                skittles2(rainbow, true);
                break;
            case 1:
                skittles2(brain, true);
                break;
            case 2:
                skittles2(garfield, true);
                break;
            case 3:
                skittles2(blizzard, true);
                break;    
            case 4:
                skittles2(oscar, true);
                break;
            case 5:
                skittles2(stitch, true);
                break;
            case 6:
                skittles2(snowballFight, true);
                break;
            case 7:
                skittles2(belle, true);
                break;
            case 8:
                skittles2(beast, true);
                break;
            case 9:
                skittles2(gaston, true);
                break;
        }
    }
    /*
    public void testMode(){
        switch(r.inputs.buttonAssignment){
            case 1:
            case 2:
                skittles2(rainbow, r.inputs.buttonAssignment % 2 > 0);
                break;
            case 3:
            case 4:
                skittles2(crabRave, r.inputs.buttonAssignment % 2 > 0);
                break;
            case 5:
            case 6:
                skittles2(snowBall, r.inputs.buttonAssignment % 2 > 0);
                break;
            case 7:
            case 8:
                skittles2(spongebob, r.inputs.buttonAssignment % 2 > 0);
                break;
            case 9:
            case 10:
                skittles2(banana, r.inputs.buttonAssignment % 2 > 0);
                break;
            case 11:
            case 12:
                skittles2(thanos, r.inputs.buttonAssignment % 2 > 0);
                break;
            case 13:
            case 14:
                skittles2(chowder, r.inputs.buttonAssignment % 2 > 0);
                break;
            case 15:
            case 16:
                skittles2(wednesday, r.inputs.buttonAssignment % 2 > 0);
                break;
            case 17:
                build(blueStripes);
                break;
            case 18:
                build(redStripes);
                break;
            default:
                build(blueStripes);
                //skittles2(rainbow, false);
        }
    }
    /* */

    int buildOffset = 0;
    public void build(Color[] colors){
        if(Timer.getFPGATimestamp() > switchTime){
            int len = ledBuffer.getLength() / 2;
            for(int i=0; i<len; i++){
                int colorIdx = ((i+offset) % (colors.length*2)) / 2;
                if(colorIdx < 0) colorIdx += colors.length;
                
                if(buildOffset > 0){
                    if(i < buildOffset) colorIdx = 0;
                } else {
                    if(i > buildOffset + len) colorIdx = 0;
                }
                
                mirrorLed(i,colors[colorIdx]);

                if(buildOffset > 0){
                    if(i == len-1){
                        if(colorIdx == 0) buildOffset++;
                        if(buildOffset == len) {
                            buildOffset = -len;
                        }
                    }
                } else {
                    if(i == 0){
                        if(colorIdx == 0) buildOffset++;
                    }
                }
                
            }
            offset++;
            switchTime = Timer.getFPGATimestamp() + 0.03;
        }
    }
    //this is the important one
    public void skittles2(Color[] colors, boolean up){
        if(Timer.getFPGATimestamp() > switchTime){
            int len = ledBuffer.getLength() / 2;
            for(int i = 0; i < len; i++){
                int idx = (i + offset) % len;
                if(idx < 0) idx += len;
                int colorIdx = ((i) % (colors.length * 2)) / 2;
                if(colorIdx < 0) colorIdx += colors.length;
                mirrorLed(idx, colors[colorIdx]);
            }
            if (up){
                offset++;
            } else {
                offset--;
            }
            switchTime = Timer.getFPGATimestamp() + 0.03;
            led.setData(ledBuffer);
        }
    }

    Color off = new Color(0,0,0);
    public void mirrorLed(int index, Color c){
        ledBuffer.setLED(index, c);
        int mirrorIdx = ledBuffer.getLength() - index - 1;
        if(/*!r.inputs.isShelf() &&*/ mirrorIdx > 60 && mirrorIdx < 78){
            ledBuffer.setLED(mirrorIdx, off);
        } else {
            ledBuffer.setLED(mirrorIdx, c);
        }
    }

    BooleanPublisher ledEnable;
    IntegerPublisher ledValue;
    int localLedValue = 0;
    public void initOutputLed(){
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        NetworkTable table = nt.getTable("ControlBoard");
        ledEnable = table.getBooleanTopic("LED_Enable").publish();
        ledValue = table.getIntegerTopic("LED_Output").publish();
    }

    public void ledOutEnable(boolean b){
        ledEnable.set(b);
    }

    public void ledOutputSet(int value, boolean on){
        int v = 1 << value;
        if(on){
            localLedValue |= v;
        } else {
            localLedValue &= ~v;
        }
        ledValue.set(localLedValue);
    }

}















/* 
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







    Color[] thanos = {
        new Color(5, 0, 10),
        new Color(15, 0, 33),
        new Color(25, 0, 45),
        new Color(50, 0, 100),
        new Color(75, 0, 125),
        new Color(100, 0, 150),
        new Color(255, 0, 255),
        new Color(255, 50, 255),
        new Color(255, 75, 255),
    };

    Color[] chowder ={
        new Color (255, 0, 255),
        new Color (255, 75, 255),
        new Color (175, 0, 190)
    };

    Color[] spongebob = {
        new Color (0, 0, 0),
        new Color (10, 3, 0),
        new Color (25, 7, 0),
        new Color (50, 20, 0),
        new Color (75, 30, 0),
        new Color (150, 40, 0),
        new Color (200, 75, 0),
        new Color (200, 75, 0),
        new Color (225, 80, 0),
        new Color (255, 100, 0),
        new Color (255, 100, 0),
    };

    Color[] banana = {
        new Color (255, 100, 0),
        new Color (200, 75, 0),
        new Color (150, 40, 0)
    };

    Color[] crabRave = {
        new Color (255, 0 ,0),
        new Color (100, 0, 0),
        new Color (255, 0, 0),
        new Color (75, 0, 0),
        new Color (255, 0, 0),
        new Color (50, 0, 0)
    };

    Color[] snowBall = {
        new Color (0, 255, 255),
        new Color (0, 0, 255),
        new Color (255, 255, 255)
    };

    Color[] wednesday = {
        new Color (200, 0, 100),
        new Color (175, 0, 75),
        new Color (150, 0, 60),
        new Color (100, 0, 40),
        new Color (75, 0, 10)
    };

    Color[] blueStripes = {
        new Color (0, 0, 200),
        new Color (0, 0, 175),
        new Color (0, 0, 150),
        new Color (0, 0, 100),
        new Color (0, 0, 50),
        new Color (0, 0, 0),
        new Color (0, 0, 0),
        new Color (0, 0, 0),
        new Color (0, 0, 0),
        new Color (0, 0, 0)
    };

    Color[] redStripes = {
        new Color (200, 0, 0),
        new Color (175, 0, 0),
        new Color (150, 0, 0),
        new Color (100, 0, 0),
        new Color (50, 0, 0),
        new Color (0, 0, 0),
        new Color (0, 0, 0),
        new Color (0, 0, 0),
        new Color (0, 0, 0),
        new Color (0, 0, 0),
    };


*/