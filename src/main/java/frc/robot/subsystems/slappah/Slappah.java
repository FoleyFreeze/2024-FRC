package frc.robot.subsystems.slappah;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.cals.SlappahCals;

public class Slappah extends SubsystemBase {

    SlappahCals k;
    RobotContainer r;

    SlappahIO io;
    SlappahIOInputsAutoLogged inputs = new SlappahIOInputsAutoLogged();

    public Slappah (RobotContainer r, SlappahCals k){
        this.k = k;
        this.r = r;

        if(Robot.isReal() && !k.disable){
            io = new SlappahIO_HW(k);
        }else{
            io = new SlappahIO(){};
        }
    }

    public boolean checkAngleError(){
        double angleSetpoint = 0;
        return Math.abs(inputs.anglePosition - angleSetpoint) < k.allowedAngleError;
    }

    public double getTransferCurrent(){
        return inputs.transferCurrentAmps;
    }

    public void setTransferPosition(double position){
        io.setPosition(position);
    }

    public void setAngle(double position){
        io.setPosition(position);
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Slappah", inputs);
    }
}
