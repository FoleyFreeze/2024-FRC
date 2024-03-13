package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        double error = 0;
        if(r.drive.inputs.navXconnected){
            error = r.drive.inputs.roll.getDegrees();
        } else {
            //use the dial if the navX is not working
            //error = r.inputs.getLeftDial() * 10 - 5;
        }
        //navx is cc pos
        //navX y-axis points backwards
        //pos cc on y-axis looks like c rot on -y axis
        //pos pitch = right side is higher then left
        double offset = error * k.balanceKP;
        offset = Math.min(Math.abs(offset), Math.abs(power)) * Math.signum(offset);
        setWinchPower(power + offset, power - offset);
    
        Logger.recordOutput("Climb/Winch", offset);
    }

    public void triggerEvenClimb(){
        if(r.inputs.gatherTriggerSWE.getAsBoolean() || r.inputs.cbConn() && !r.inputs.shiftB6.getAsBoolean() && r.inputs.climbWinchB2.getAsBoolean()){
            evenClimb(k.climbUpPwr);
        }else if(r.inputs.cbConn() && r.inputs.shiftB6.getAsBoolean() && r.inputs.climbWinchB2.getAsBoolean()){
            evenClimb(k.climbDownPwr);
        }else{
            setWinchPower(0, 0);
        }
    }

    public void setTestPower(){
        double power = r.inputs.getRightDial();
        if (r.inputs.SWBHi.getAsBoolean()){
            //do nothing
        }else if(r.inputs.SWBLo.getAsBoolean()){
            power = -power;
        }else{
            power = 0;
        }
        evenClimb(power);
        //setWinchPower(power, power);

    }

    public void winchUp(){
        setWinchPower(k.climbUpPwr, k.climbUpPwr);
    }

    public void winchDown(){
        setWinchPower(k.climbDownPwr, k.climbDownPwr);
    }

    public void winchHold(){
        io.setWinchPosition(inputs.winchLPosition, inputs.winchRPosition);
    }

    public void winchJogLeft(){
        setpointL += k.jogWinchAmount;
        setpointR -= k.jogWinchAmount;
    }

    public void winchJogRight(){
        setpointL -= k.jogWinchAmount;
        setpointR += k.jogWinchAmount;
    }

    public void setBrakes(boolean on){
        io.setBrakes(on);
    }


    double setpointL;
    double setpointR;
    double winchStartL;
    double winchStartR;

    public void captureSetpoints(){
        winchStartL = inputs.winchLPosition;
        winchStartR = inputs.winchRPosition;
    }

    public void setWinchPosition(double delta){
        setpointL = winchStartL + delta;
        setpointR = winchStartR + delta;
        io.setWinchPosition(setpointL, setpointR);
    }

    public boolean checkWinchPosition(){
        return Math.abs(inputs.winchLPosition - setpointL)
             + Math.abs(inputs.winchRPosition - setpointR)
             < 0.125;
    }

    public void setWinchPower(double leftPower, double rightPower){
        io.setWinchVoltage(leftPower*12, rightPower*12);
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

        double power = r.inputs.getRightDial(); 
        if(r.inputs.SWBLo.getAsBoolean()){
            power = -power;
        }
        SmartDashboard.putNumber("Climb Power", power);
        SmartDashboard.putBoolean("ClimbEnabled", r.inputs.SWBHi.getAsBoolean() || r.inputs.SWBLo.getAsBoolean());
    }
}
