package frc.robot.cals;

import frc.robot.subsystems.motor.MotorCal;
import frc.robot.subsystems.motor.MotorCal.TypeMotor;

public class GatherCals {
    public boolean disable = false;
    
    //public MotorCal topGather = new MotorCal(TypeMotor.SPARK, 14)
    public MotorCal topGather = new MotorCal(TypeMotor.SPARK, 7)
        .setRatio(0.5)
        .setCurrLim(60, 80)
        .invert()
        .setRampRate(0.1);
    
    public MotorCal gateMotor = new MotorCal(TypeMotor.SPARK, 12)
        .setRatio(1)
        .setCurrLim(40)
        .setPIDF(0.3, 0.001, 0, 0)
        .setIZone(0.25)
        .setPIDPwrLim(0.2)
        .setRampRate(0.1)
        .setBrakeMode(true);
}
