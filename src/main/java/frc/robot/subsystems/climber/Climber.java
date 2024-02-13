package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.cals.ClimberCals;

public class Climber extends SubsystemBase{

    ClimberCals k;
    RobotContainer r;

    ClimberIO io;
    ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber (RobotContainer r, ClimberCals k){
        this.r = r;
        this.k = k;

        if(Robot.isReal() && !k.disabled){
            io = new ClimberIO_HW(k);
        }else{
            io = new ClimberIO(){};
        }
    }
    public void balancedClimb(double power){
        //TODO:This. ¯\_(ツ)_/¯
    }

    public double getCurrent(){
        return inputs.winchRCurrentAmps + inputs.winchLCurrentAmps;
    }

    public double getWinchRCurrent(){
        return inputs.winchRCurrentAmps;
    }

    public double getWinchLCurrent(){
        return inputs.winchLCurrentAmps;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

    }
}
