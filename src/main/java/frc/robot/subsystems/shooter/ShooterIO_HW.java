package frc.robot.subsystems.shooter;

import frc.robot.cals.ShooterCals;
import frc.robot.subsystems.motor.Motor;

public class ShooterIO_HW implements ShooterIO {

    ShooterCals k;

    Motor angleMotor;
    Motor shootMotorTop;
    Motor shootMotorBottom;

    boolean negativePowerEnabled = false;

    public ShooterIO_HW (ShooterCals k){
        angleMotor = Motor.create(k.angleMotor);
        shootMotorTop = Motor.create(k.shootMotorTop);
        shootMotorBottom = Motor.create(k.shootMotorBottom);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs){
        int errorCount = shootMotorBottom.getErrorCount();
        double position = shootMotorBottom.getPosition();
        double velocity = shootMotorBottom.getVelocity();
        double current = shootMotorBottom.getCurrent();
        double voltage = shootMotorBottom.getVoltage();
        double temp = shootMotorBottom.getTemp();

        if(shootMotorBottom.getErrorCount() == errorCount){
            inputs.shootBottomPosition = position;
            inputs.shootBottomVelocity = velocity;
            inputs.shootBottomCurrentAmps = current;
            inputs.shootBottomAppliedVolts = voltage;
            inputs.shootBottomTemp = temp;
        }

        errorCount = shootMotorTop.getErrorCount();
        position = shootMotorTop.getPosition();
        velocity = shootMotorTop.getVelocity();
        current = shootMotorTop.getCurrent();
        voltage = shootMotorTop.getVoltage();
        temp = shootMotorTop.getTemp();

        if(shootMotorTop.getErrorCount() == errorCount){
            inputs.shootTopPosition = position;
            inputs.shootTopVelocity = velocity;
            inputs.shootTopCurrentAmps = current;
            inputs.shootTopAppliedVolts = voltage;
            inputs.shootTopTemp = temp;
        }

        errorCount = angleMotor.getErrorCount();
        position = angleMotor.getPosition();
        velocity = angleMotor.getVelocity();
        current = angleMotor.getCurrent();
        voltage = angleMotor.getVoltage();
        temp = angleMotor.getTemp();

        if(angleMotor.getErrorCount() == errorCount){
            inputs.anglePosition = position;
            inputs.angleVelocity = velocity;
            inputs.angleCurrentAmps = current;
            inputs.angleAppliedVolts = voltage;
            inputs.angleTemp = temp;
        }
    }

    @Override
    public void setShooterVoltage(double voltage){
        if(voltage < 0 && !negativePowerEnabled){
            negativePowerEnabled = true;
            shootMotorTop.setPIDPwrLim(1);
            shootMotorBottom.setPIDPwrLim(1);
            System.out.println("Enable Falcon negative power");
        }
        shootMotorTop.setVoltage(voltage);
        shootMotorBottom.setVoltage(voltage);
    }

    @Override
    public void setShooterRPM(double rpm){
        if(negativePowerEnabled){
            negativePowerEnabled = false;
            shootMotorTop.setPIDPwrLim(1, 0);
            shootMotorBottom.setPIDPwrLim(1, 0);
            System.out.println("Disable Falcon negative power");
        }
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
