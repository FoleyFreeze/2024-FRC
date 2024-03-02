package frc.robot.subsystems.slappah;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.cals.SlappahCals;

public class Slappah extends SubsystemBase {

    SlappahCals k;
    RobotContainer r;

    SlappahIO io;
    SlappahIOInputsAutoLogged inputs = new SlappahIOInputsAutoLogged();

    double angleSetpoint;
    double transferSetPoint;

    public Slappah (RobotContainer r, SlappahCals k){
        this.k = k;
        this.r = r;

        if(Robot.isReal() && !k.disable){
            io = new SlappahIO_HW(k);
        }else{
            io = new SlappahIO(){};
        }

        io.setArmEncoderPosition(k.startAngle);
        angleSetpoint = k.startAngle;
    }

    public boolean checkAngleError(){
        return Math.abs(inputs.anglePosition - angleSetpoint) < k.allowedAngleError;
    }

    public boolean checkTransferError(){
        return Math.abs(inputs.transferPosition - transferSetPoint) < k.allowedTransferError;
    }

    public void setTransferPower(double power){
        io.setTransferVoltage(power*12);
    }

    public double getTransferCurrent(){
        return inputs.transferCurrentAmps;
    }

    public void setTransferPosition(double delta){
        io.setPosition(inputs.transferPosition + delta);
    }

    public void setAngle(double position){
        angleSetpoint = position;
        io.setPosition(position);
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Slappah", inputs);

        SmartDashboard.putNumber("Slappah Angle", inputs.anglePosition);
    }
}
