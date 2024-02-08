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
        //make this
    }

    @Override
    public void setShooterVoltage(double voltage){
        shootMotorTop.setVoltage(voltage);
        shootMotorBottom.setVoltage(voltage);
    }


    @Override
    public void setAngle(double angle){
        //make this
    }

}
