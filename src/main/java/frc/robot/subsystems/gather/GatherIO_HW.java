package frc.robot.subsystems.gather;

import frc.robot.cals.GatherCals;
import frc.robot.subsystems.motor.Motor;

public class GatherIO_HW implements GatherIO {
    
    Motor topGather;
    Motor gateMotor;
    
    public GatherIO_HW (GatherCals k){
        topGather = Motor.create(k.topGather);
        gateMotor = Motor.create(k.gateMotor);
    }

    @Override
    public void updateInputs (GatherIOInputs inputs){
        inputs.intakePosition = topGather.getPosition();
        inputs.intakeVelocity = topGather.getVelocity();
        inputs.intakeCurrentAmps = topGather.getCurrent();
        inputs.intakeAppliedVolts = topGather.getVoltage();
    }

    @Override
    public void setIntakeVoltage(double voltage){
        topGather.setVoltage(voltage);
    }

    @Override
    public void setGateVoltage(double gateVoltage){
        gateMotor.setVoltage(gateVoltage);
    }

    @Override 
    public void setGatePosition(double positon){
        gateMotor.setPosition(positon);
    }

}
