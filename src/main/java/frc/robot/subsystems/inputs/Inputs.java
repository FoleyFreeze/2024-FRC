package frc.robot.subsystems.inputs;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        double x = flysky.getRawAxis(0);
        double y = flysky.getRawAxis(1);
        double z = flysky.getRawAxis(4);
        
        x = deadband(x);
        y = deadband(y);
        z = deadband(z);

        x = Math.pow(x, k.expo);
        y = Math.pow(y, k.expo);
        z = Math.pow(z, k.expo);

        return new ChassisSpeeds(x, y, z);
    }

    public double deadband(double value){
        if(value > k.deadband){
            return (value - k.deadband) / (1-k.deadband);
        }
        else if(value < -k.deadband){
            return (value + k.deadband) / (1-k.deadband);
        }else return 0;
    }
}
