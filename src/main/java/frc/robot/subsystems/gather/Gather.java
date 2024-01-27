package frc.robot.subsystems.gather;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.cals.GatherCals;
import org.littletonrobotics.junction.Logger;

public class Gather extends SubsystemBase{
    
    GatherCals k;
    RobotContainer r;

    GatherIO io;
    GatherIOInputsAutoLogged inputs;
    

    public Gather (RobotContainer r, GatherCals k){
        this.k = k;
        this.r = r;

        if(Robot.isReal()){
            io = new GatherIO_HW(k);
        }else{
            io = new GatherIO(){};
        }
    }

    public void setMotorPower(double power){
        io.setGatherVoltage(power*12);
        Logger.recordOutput("Gather/SetpointVoltage", power*12);
    }

    
    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Gather/TopGather", inputs);
    }
}
