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
    public SlappahIOInputsAutoLogged inputs = new SlappahIOInputsAutoLogged();

    double angleSetpoint;
    double transferSetPoint;

    public double angleJog = 0;

    public Slappah (RobotContainer r, SlappahCals k){
        this.k = k;
        this.r = r;

        if(Robot.isReal() && !k.disable){
            io = new SlappahIO_HW(k);
        }else{
            io = new SlappahIO(){};
        }

        io.updateInputs(inputs);
        Logger.processInputs("Slappah", inputs);        
        if(inputs.anglePosition < 0.1){ 
            io.setArmEncoderPosition(k.startAngle);
            System.out.println("Successfully reset slap angle position from: " + inputs.anglePosition);
            angleSetpoint = k.startAngle;
        } else {
            System.out.println("Did not need to reset slapper angle. Already at: " + inputs.anglePosition);
            angleSetpoint = inputs.anglePosition;
        }
    }

    public void resetArmAngle(){
        io.setArmEncoderPosition(k.startAngle);
        System.out.println("Zero'd Arm");
    }

    public void setBrake(boolean on){
        io.setAngleBrake(on);
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
        transferSetPoint = inputs.transferPosition + delta;
        io.setTransferPosition(transferSetPoint);
    }

    public void setAngle(double position){
        angleSetpoint = position;
        io.setPosition(position + angleJog);
    }

    public void jogAngle(double jog){
        angleJog += jog;
        System.out.println("Arm Angle Jog is: " + angleJog);
        setAngle(angleSetpoint);
    }

    public void setAnglePwr(double power){
        io.setAngleVoltage(power*12);
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Slappah", inputs);

        SmartDashboard.putNumber("Slappah Angle", inputs.anglePosition);
        SmartDashboard.putNumber("Slappah Temp", inputs.angleTemp);
    }
}
