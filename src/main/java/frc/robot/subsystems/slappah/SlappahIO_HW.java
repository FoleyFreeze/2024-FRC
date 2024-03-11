package frc.robot.subsystems.slappah;

import frc.robot.cals.SlappahCals;
import frc.robot.subsystems.motor.Motor;

public class SlappahIO_HW implements SlappahIO {
    
    Motor transfer;
    Motor angle;

    public SlappahIO_HW (SlappahCals k){
        transfer = Motor.create(k.transferMotor);
        angle = Motor.create(k.angleMotor);
    }

    @Override
    public void updateInputs(SlappahIOInputs inputs){
        int errorCount = transfer.getErrorCount();
        double position = transfer.getPosition();
        double velocity = transfer.getVelocity();
        double current = transfer.getCurrent();
        double voltage = transfer.getVoltage();
        double temp = transfer.getTemp();

        if(transfer.getErrorCount() == errorCount){
            inputs.transferPosition = position;
            inputs.transferVelocity = velocity;
            inputs.transferCurrentAmps = current;
            inputs.transferAppliedVolts = voltage;
            inputs.transferTemp = temp;
        }

        errorCount = angle.getErrorCount();
        position = angle.getPosition();
        velocity = angle.getVelocity();
        current = angle.getCurrent();
        voltage = angle.getVoltage();
        temp = angle.getTemp();

        if(angle.getErrorCount() == errorCount){
            inputs.anglePosition = position;
            inputs.angleVelocity = velocity;
            inputs.angleCurrentAmps = current;
            inputs.angleAppliedVolts = voltage;
            inputs.angleTemp = temp;
        }
    }

    @Override
    public void setPosition(double position){
        angle.setPosition(position);
    }

    @Override
    public void setAngleVoltage(double voltage){
        angle.setVoltage(voltage);
    }

    @Override
    public void setArmEncoderPosition(double position){
        angle.setEncoderPosition(position);
    }

    @Override
    public void setAngleBrake(boolean on){
        angle.setBrakeMode(on);
    }

    @Override 
    public void setTransferVoltage(double voltage){
        transfer.setVoltage(voltage);
    }

    @Override
    public void setTransferPosition(double position){
        transfer.setPosition(position);
    }

}
