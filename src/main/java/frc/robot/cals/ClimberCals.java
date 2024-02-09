package frc.robot.cals;

import frc.robot.subsystems.motor.MotorCal;
import frc.robot.subsystems.motor.MotorCal.TypeMotor;

public class ClimberCals {
    public boolean disable = false;

    public MotorCal winchR = new MotorCal(TypeMotor.SPARK, 0);
    public MotorCal winchL = new MotorCal(TypeMotor.SPARK, 0);

    public boolean disabled;
}
