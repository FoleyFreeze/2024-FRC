package frc.robot.subsystems.gather;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.cals.GatherCals;
import frc.robot.subsystems.motor.Motor;

public class GatherIO_HW implements GatherIO {
    
    Motor topGather;
    Motor gateMotor;
    
    DigitalInput proxSw;

    public GatherIO_HW (GatherCals k){
        topGather = Motor.create(k.topGather);
        gateMotor = Motor.create(k.gateMotor);

        proxSw = new DigitalInput(0);
    }

    @Override
    public void updateInputs (GatherIOInputs inputs){
        int errorCount = topGather.getErrorCount();
        double position = topGather.getPosition();
        double velocity = topGather.getVelocity();
        double current = topGather.getCurrent();
        double voltage = topGather.getVoltage();
        double temp = topGather.getTemp();

        if(topGather.getErrorCount() == errorCount){
            inputs.intakePosition = position;
            inputs.intakeVelocity = velocity;
            inputs.intakeCurrentAmps = current;
            inputs.intakeAppliedVolts = voltage;
            inputs.intakeAppliedVolts = temp;
        }

        errorCount = gateMotor.getErrorCount();
        position = gateMotor.getPosition();
        velocity = gateMotor.getVelocity();
        current = gateMotor.getCurrent();
        voltage = gateMotor.getVoltage();
        temp = gateMotor.getTemp();

        if(gateMotor.getErrorCount() == errorCount){
            inputs.gatePosition = position;
            inputs.gateVelocity = velocity;
            inputs.gateCurrentAmps = current;
            inputs.gateAppliedVolts = voltage;
            inputs.gateTemp = temp;
        }

        inputs.proxSensor = !proxSw.get();
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
