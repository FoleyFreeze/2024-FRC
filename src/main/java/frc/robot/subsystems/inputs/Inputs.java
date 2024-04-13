package frc.robot.subsystems.inputs;

import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.cals.InputsCals;

public class Inputs extends SubsystemBase {
    
    public Joystick flysky;
    RobotContainer r;
    InputsCals k;

    public Trigger gatherTriggerSWE = new Trigger(() -> flysky.getRawAxis(2) > .25);
    public Trigger shootTriggerSWH = new Trigger(() -> flysky.getRawAxis(3) > .25);
    public Trigger cameraEnableSWD = new Trigger(() -> flysky.getRawButton(5));
    public Trigger fieldOrientedSWA = new Trigger(() -> flysky.getRawButton(1));
    public Trigger resetFieldOrientedLTRIM = new Trigger(() -> flysky.getRawButton(10));
    public Trigger resetFieldOdometryRTRIM = new Trigger(() -> flysky.getRawButton(14));
    public Trigger resetArmLRTRIMR = new Trigger(() -> flysky.getRawButton(12) && flysky.getRawButton(16));
    public Trigger resetShooterLRTRIML = new Trigger(() -> flysky.getRawButton(13) && flysky.getRawButton(17));

    public Trigger SWBHi = new Trigger(() -> flysky.getRawButton(2));
    public Trigger SWBLo = new Trigger(() -> flysky.getRawButton(3));
    public Trigger SWC = new Trigger(() -> flysky.getRawButton(4));
    public Trigger SWEHi = new Trigger(() -> flysky.getRawButton(7));
    public Trigger SWELo = new Trigger(() -> flysky.getRawButton(6));
    public Trigger SWGHi = new Trigger(() -> flysky.getRawButton(9));
    public Trigger SWGLo = new Trigger(() -> flysky.getRawButton(8));

    public Joystick controlBoard1;
    public Joystick controlBoard2;

    public Trigger shiftB6 = new Trigger(() -> cbConn() && controlBoard1.getRawButton(6));
    public Trigger shootAngleJogUp = new Trigger(() -> cbConn() && controlBoard1.getPOV() == 0);
    public Trigger shootAngleJogDn = new Trigger(() -> cbConn() && controlBoard1.getPOV() == 180);
    public Trigger armAngleJogUp = new Trigger(() -> cbConn() && controlBoard1.getPOV() == 90);
    public Trigger armAngleJogDn = new Trigger(() -> cbConn() && controlBoard1.getPOV() == 270);

    public Trigger climbDeployB4 = new Trigger(() -> cbConn() && controlBoard1.getRawButton(4));
    public Trigger climbWinchB2 = new Trigger(() -> cbConn() && controlBoard1.getRawButton(2));
    public Trigger gatherBtnB5 = new Trigger(() -> cbConn() && controlBoard1.getRawButton(5));
    public Trigger transferB3 = new Trigger(() -> cbConn() && controlBoard1.getRawButton(3));
    public Trigger shootBtnB1 = new Trigger(() -> cbConn() && controlBoard1.getRawButton(1));
    public Trigger buddyBtnB1 = new Trigger(() -> cbConn() && controlBoard2.getRawButton(1));

    public Inputs (RobotContainer r, InputsCals k){
        this.r = r;
        this.k = k;
        
        flysky = new Joystick(0);
        controlBoard1 = new Joystick(1);
        controlBoard2 = new Joystick(2);
    }

    public boolean cbConn(){
        return controlBoard1.isConnected();
    }

    public ChassisSpeeds getChassisSpeeds(){
        double y = -flysky.getRawAxis(0);
        double x = -flysky.getRawAxis(1);
        double z = -flysky.getRawAxis(4);
        
        if(Math.abs(x) < k.xyDeadband && Math.abs(y) < k.xyDeadband){
            x = 0;  
            y = 0;
        }

        z = deadband(z, k.omegaDeadband);

        x = Math.pow(Math.abs(x), k.expo) * Math.signum(x);
        y = Math.pow(Math.abs(y), k.expo) * Math.signum(y);
        z = Math.pow(Math.abs(z), k.expo) * Math.signum(z);

        Logger.recordOutput("Drive/Inputs y", y);

        Translation2d newXY = mapSqrToCirc(x, y);
        x = newXY.getX();
        y = newXY.getY();

        x *= k.maxDrivePwr;
        y *= k.maxDrivePwr;
        z *= k.maxDrivePwr;

        ChassisSpeeds speeds = new ChassisSpeeds(x, y, z);
        Logger.recordOutput("Drive/Inputs", speeds);
        return speeds;
    }

    public Translation2d mapSqrToCirc(double x, double y){
        double length1 = Math.sqrt((x*x)+(y*y));
        double theta = Math.atan2(y, x);
        double slope;

        /*
        if (Math.abs(theta % Math.PI/2) > Math.PI/4){
            slope = x/y;
            if(Math.abs(y) < 0.01) slope = 1;
        }else{
            slope = y/x;
            if(Math.abs(x) < 0.01) slope = 1;
        }
        */

        if(Math.abs(x) < 0.01 && Math.abs(y) < 0.01){
            slope = 1;
        }
        else if(Math.abs(x) > Math.abs(y)){
            slope = y / x;
        } else {
            slope = x / y;
        }


        double length2 = Math.sqrt(1 + slope*slope);
        double r = length1/length2;

        return new Translation2d((r*Math.cos(theta)), r*Math.sin(theta));
    }

    public double deadband(double value, double deadband){
        /*
        if(value > k.deadband){
            return (value - k.deadband) / (1-k.deadband);
        }
        else if(value < -k.deadband){
            return (value + k.deadband) / (1-k.deadband);
        }else return 0;
        */
        if(Math.abs(value) < deadband){
            return 0;
        } else {
            return value;
        }
    }

    public Trigger resetSwerveZerosTRIM2DN = new Trigger(new BooleanSupplier() {
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
        if(SWGLo.getAsBoolean()){
            return 2;
        } else if(SWGHi.getAsBoolean()){
            return 0;
        }else {
            return 1;
        }
    }

    public int getClimbDir(){
        if(controlBoard2.getRawButton(3)){
            return 1;//right
        } else if(controlBoard2.getRawButton(2)) {
            return -1;//left
        } else {
            return 0;
        }
    }

    public boolean getPitMode(){
        return controlBoard2.getRawButton(1);
    }

    public double getLeftDial(){
        return (flysky.getRawAxis(5) + 1) / 2.0;
    }

    public double getRightDial(){
        return (flysky.getRawAxis(6) + 1) / 2.0;
    }
 
    @Override
    public void periodic() {
        // Called once per scheduler run
        
    }
}
