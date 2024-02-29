package frc.robot.cals;

import frc.robot.subsystems.motor.MotorCal;
import frc.robot.subsystems.motor.MotorCal.TypeMotor;

public class ClimberCals {
    public boolean disable = true;

    //twenty percent power at 10 degrees
    public double balanceKP = .2/10;

    public MotorCal winchR = new MotorCal(TypeMotor.SPARK, 0);
    public MotorCal winchL = new MotorCal(TypeMotor.SPARK, 0);
}
