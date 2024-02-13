package frc.robot.subsystems.inputs;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.cals.InputsCals;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Inputs extends SubsystemBase {
    
    public Joystick flysky;
    RobotContainer r;
    InputsCals k;

    public Trigger gatherTrigger = new Trigger(() -> flysky.getRawAxis(2) > .25);
    public Trigger shootTrigger = new Trigger(() -> flysky.getRawAxis(3) > .25);
    public Trigger cameraEnable = new Trigger(() -> flysky.getRawButton(5));
    public Trigger fieldOriented = new Trigger(() -> flysky.getRawButton(1));
    public Trigger resetFieldOriented = new Trigger(() -> flysky.getRawButton(10));
    public Trigger resetFieldOdometry = new Trigger(() -> flysky.getRawButton(14));

    public Joystick controlBoard;

    public Trigger shootAngleJogUp = new Trigger(() -> controlBoard.getRawButton(0));
    public Trigger shootAngleJogDn = new Trigger(() -> controlBoard.getRawButton(0));
    public Trigger shift = new Trigger(() -> controlBoard.getRawButton(0));

    public Inputs (RobotContainer r, InputsCals k){
        this.r = r;
        this.k = k;
        
        flysky = new Joystick(0);
        //controlBoard = new Joystick(1);
    }

    public ChassisSpeeds getChassisSpeeds(){
        double y = -flysky.getRawAxis(0);
        double x = -flysky.getRawAxis(1);
        double z = -flysky.getRawAxis(4);
        
        if(Math.abs(x) < k.deadband && Math.abs(y) < k.deadband){
            x = 0;  
            y = 0;
        }

        z = deadband(z);

        /*
        x = Math.pow(x, k.expo) * Math.signum(x);
        y = Math.pow(y, k.expo) * Math.signum(y);
        z = Math.pow(z, k.expo) * Math.signum(z);
        

        Translation2d newXY = mapSqrToCirc(x, y);
        x = newXY.getX();
        y = newXY.getY();
        */

        x *= k.maxDrivePwr;
        y *= k.maxDrivePwr;
        z *= k.maxDrivePwr;

        return new ChassisSpeeds(x, y, z);
    }

    public Translation2d mapSqrToCirc(double x, double y){
        double length1 = Math.sqrt((x*x)+(y*y));
        double theta = Math.atan2(y, x);
        double slope;

        if (Math.abs(theta % Math.PI/2) > Math.PI/4){
            slope = x/y;
            if(y == 0) slope = 1;
        }else{
            slope = y/x;
            if(x == 0) slope = 1;
        }

        double length2 = Math.sqrt(1 + slope*slope);
        double r = length1/length2;

        return new Translation2d((r*Math.cos(theta)), r*Math.sin(theta));
    }

    public double deadband(double value){
        /*
        if(value > k.deadband){
            return (value - k.deadband) / (1-k.deadband);
        }
        else if(value < -k.deadband){
            return (value + k.deadband) / (1-k.deadband);
        }else return 0;
        */
        if(Math.abs(value) < k.deadband){
            return 0;
        } else {
            return value;
        }
    }

    public Trigger resetSwerveZeros = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(flysky != null){
                return flysky.getRawButton(11) && flysky.getRawButton(15);
            } else {
                return false;
            }
        }
    });

    public int getFixedTarget(){
        //8 down 9 up
        //right triggr 3pos switch 
        //up is layup mid is freethrow down is threepoint
        if(flysky.getRawButton(8)){
            return 0;
        } else if(flysky.getRawButton(9)){
            return 2;
        }else {return 1;}

    }
 
    @Override
    public void periodic() {
        // Called once per scheduler run
        
    }
}
