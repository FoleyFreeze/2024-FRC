package frc.robot.subsystems.gather;

import frc.robot.cals.GatherCals;
import frc.robot.subsystems.motor.Motor;

public class GatherIO_HW implements GatherIO {
    
    Motor topGather;
    
    public GatherIO_HW (GatherCals k){
        topGather = Motor.create(k.topGather);
    }

    @Override
    public void updateInputs (GatherIOInputs inputs){
        inputs.gatherPosition = topGather.getPosition();
        inputs.gatherVelocity = topGather.getVelocity();
        inputs.gatherCurrentAmps = topGather.getCurrent();
        inputs.gatherAppliedVolts = topGather.getVoltage();
    }

    @Override
    public void setGatherVoltage(double voltage){
        topGather.setVoltage(voltage);
    }
}
