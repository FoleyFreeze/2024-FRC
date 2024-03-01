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

        if(Robot.isReal() && !k.disable){
            io = new ClimberIO_HW(k);
        }else{
            io = new ClimberIO(){};
        }
    }
    public void evenClimb(double power){
        double error = r.drive.inputs.roll.getDegrees();
        //navx is cc pos
        //navX y-axis points backwards
        //pos cc on y-axis looks like c rot on -y axis
        //pos roll = right side is higher then left
        double offset = error * k.balanceKP;
        offset = Math.min(Math.abs(offset), Math.abs(power)) * Math.signum(offset);
        setWinchPower(power + offset, power - offset);
    
        Logger.recordOutput("Climb/Winch", offset);
    }

    public void setTestPower(){
        
    }

    public void setWinchPower(double leftPower, double rightPower){
        io.setWinchVoltage(leftPower, rightPower);
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
