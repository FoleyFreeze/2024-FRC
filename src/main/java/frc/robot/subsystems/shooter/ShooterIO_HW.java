package frc.robot.subsystems.shooter;

import frc.robot.cals.ShooterCals;
import frc.robot.subsystems.motor.Motor;

public class ShooterIO_HW implements ShooterIO {

    ShooterCals k;

    Motor angleMotor;
    Motor shootMotorTop;
    Motor shootMotorBottom;

    public ShooterIO_HW (ShooterCals k){
        angleMotor = Motor.create(k.angleMotor);
        shootMotorTop = Motor.create(k.shootMotorTop);
        shootMotorBottom = Motor.create(k.shootMotorBottom);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs){
        inputs.shootBottomPosition = shootMotorBottom.getPosition();
        inputs.shootBottomVelocity = shootMotorBottom.getVelocity();
        inputs.shootBottomCurrentAmps = shootMotorBottom.getCurrent();
        inputs.shootBottomAppliedVolts = shootMotorBottom.getVoltage();

        inputs.shootTopPosition = shootMotorTop.getPosition();
        inputs.shootTopVelocity = shootMotorTop.getVelocity();
        inputs.shootTopCurrentAmps = shootMotorTop.getCurrent();
        inputs.shootTopAppliedVolts = shootMotorTop.getVoltage();

        inputs.anglePosition = angleMotor.getPosition();
        inputs.angleVelocity = angleMotor.getVelocity();
        inputs.angleCurrentAmps = angleMotor.getCurrent();
        inputs.angleAppliedVolts = angleMotor.getVoltage();
    }

    @Override
    public void setShooterVoltage(double voltage){
        shootMotorTop.setVoltage(voltage);
        shootMotorBottom.setVoltage(voltage);
    }

    @Override
    public void setShooterRPM(double rpm){
        shootMotorTop.setVelocity(rpm);
        shootMotorBottom.setVelocity(rpm);
    }

    @Override
    public void setAngle(double angle){
        angleMotor.setPosition(angle);
    }

    @Override
    public void setAngleEncoder(double angle){
        angleMotor.setEncoderPosition(angle);
    }

}
