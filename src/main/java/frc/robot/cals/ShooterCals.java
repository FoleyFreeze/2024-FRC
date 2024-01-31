package frc.robot.cals;

import frc.robot.subsystems.motor.MotorCal;
import frc.robot.subsystems.motor.MotorCal.TypeMotor;

public class ShooterCals {
    public boolean disable = true;
    
    public MotorCal angleMotor = new MotorCal(TypeMotor.SPARK, 0);
    public MotorCal gateMotor = new MotorCal(TypeMotor.SPARK, 0);
    public MotorCal shootMotorTop = new MotorCal(TypeMotor.SPARK, 0);
    public MotorCal shootMotorBottom = new MotorCal(TypeMotor.SPARK, 0);
}    