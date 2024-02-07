package frc.robot.cals;

import frc.robot.subsystems.motor.MotorCal;
import frc.robot.subsystems.motor.MotorCal.TypeMotor;

public class GatherCals {
    public boolean disable = false;
    
    public MotorCal topGather = new MotorCal(TypeMotor.SPARK, 14).setCurrLim(30).invert().setRatio(0.5);
    public MotorCal gateMotor = new MotorCal(TypeMotor.SPARK, 0).setRatio(0.5);

}
