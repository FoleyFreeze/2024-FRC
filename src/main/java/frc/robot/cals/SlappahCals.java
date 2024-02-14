package frc.robot.cals;

import frc.robot.subsystems.motor.MotorCal;
import frc.robot.subsystems.motor.MotorCal.TypeMotor;

public class SlappahCals {
    public boolean disable = true;

    public MotorCal angleMotor = new MotorCal(TypeMotor.SPARK, 0);
    public MotorCal transferMotor = new MotorCal(TypeMotor.SPARK, 0);

    public double allowedAngleError;
}
