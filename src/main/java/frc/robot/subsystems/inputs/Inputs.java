package frc.robot.subsystems.inputs;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.cals.InputsCals;

public class Inputs extends SubsystemBase {
    
    Joystick flysky;
    RobotContainer r;
    InputsCals k;

    public Inputs (RobotContainer r, InputsCals k){
        flysky = new Joystick(0);

        this.r = r;
        this.k = k;
    }

    public ChassisSpeeds getChassisSpeeds(){
        double y = -flysky.getRawAxis(0);
        double x = -flysky.getRawAxis(1);
        double z = -flysky.getRawAxis(4);
        
        x = deadband(x);
        y = deadband(y);
        z = deadband(z);

        /*
        x = Math.pow(x, k.expo) * Math.signum(x);
        y = Math.pow(y, k.expo) * Math.signum(y);
        z = Math.pow(z, k.expo) * Math.signum(z);
        */

        x *= k.maxDrivePwr;
        y *= k.maxDrivePwr;
        z *= k.maxDrivePwr;

        return new ChassisSpeeds(x, y, z);
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
                return flysky.getRawButton(11);
            } else {
                return false;
            }
        }
    });
}
