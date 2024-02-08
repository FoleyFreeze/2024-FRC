package frc.robot.cals;

import frc.robot.subsystems.motor.MotorCal;
import frc.robot.subsystems.motor.MotorCal.TypeMotor;

public class GatherCals {
    public boolean disable = false;
    
    //public MotorCal topGather = new MotorCal(TypeMotor.SPARK, 14)
    public MotorCal topGather = new MotorCal(TypeMotor.SPARK, 5)
        .setRatio(0.5)
        .setCurrLim(30)
        .invert()
        .setRampRate(0.1);
    
    public MotorCal gateMotor = new MotorCal(TypeMotor.SPARK, 6)
        .setCurrLim(30)
        .setPIDF(0.3, 0.005, 0, 0)
        .setIZone(1)
        .setPIDPwrLim(0.2)
        .setRampRate(0.1);
}
