package frc.robot.subsystems.gather;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.cals.GatherCals;
import org.littletonrobotics.junction.Logger;

public class Gather extends SubsystemBase{
    
    GatherCals k;
    RobotContainer r;

    GatherIO io;
    GatherIOInputsAutoLogged inputs = new GatherIOInputsAutoLogged();
    

    public Gather (RobotContainer r, GatherCals k){
        this.k = k;
        this.r = r;

        if(Robot.isReal() && !k.disable){
            io = new GatherIO_HW(k);
        }else{
            io = new GatherIO(){};
        }
    }

    public void setGatherPower(double intakePower, double gatePower){
        setGatePower(gatePower);
        setIntakePower(intakePower);
    }

    public void setIntakePower(double intakePower){
        io.setIntakeVoltage(intakePower*12);
        Logger.recordOutput("Gather/IntakeSetpointVoltage", intakePower*12);
    }

    public void setGatePower(double gatePower){
        io.setGateVoltage(gatePower*12);
        Logger.recordOutput("Gather/GateSetpointVoltage", gatePower*12);
    }

    public double getCurrent(){
        return inputs.intakeCurrentAmps;
    }

    public double getGateCurrent(){
        return inputs.gateCurrentAmps;
    }

    public void setGatePosition(double delta){
        io.setGatePosition(inputs.gatePosition + delta);
    }

    
    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Gather", inputs);

        SmartDashboard.putNumber("GatherCurrent", inputs.intakeCurrentAmps);
        SmartDashboard.putNumber("Setpoint", r.inputs.flysky.getRawAxis(7)/2+0.5);
    }
}
