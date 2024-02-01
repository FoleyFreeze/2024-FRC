package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.cals.ShooterCals;

public class Shooter extends SubsystemBase {
    ShooterCals k;
    RobotContainer r;

    ShooterIO io;
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    
    public Shooter (RobotContainer r, ShooterCals k){

        this.k = k;
        this.r = r;

        if(Robot.isReal() && !k.disable){
            io = new ShooterIO_HW(k);
        }else{
            io = new ShooterIO(){};
        }
    }
}
