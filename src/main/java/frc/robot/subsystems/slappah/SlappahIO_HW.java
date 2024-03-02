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
        inputs.transferPosition = transfer.getPosition();
        inputs.transferVelocity = transfer.getVelocity();
        inputs.transferCurrentAmps = transfer.getCurrent();
        inputs.transferAppliedVolts = transfer.getVoltage();
        inputs.transferTemp = transfer.getTemp();

        inputs.anglePosition = angle.getPosition();
        inputs.angleVelocity = angle.getVelocity();
        inputs.angleCurrentAmps = angle.getCurrent();
        inputs.angleAppliedVolts = angle.getVoltage();
        inputs.angleTemp = angle.getTemp();
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
    public void setTransferVoltage(double voltage){
        transfer.setVoltage(voltage);
    }

    @Override
    public void setTransferPosition(double position){
        transfer.setPosition(position);
    }

}
