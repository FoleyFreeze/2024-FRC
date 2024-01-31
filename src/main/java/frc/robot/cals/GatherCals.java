package frc.robot.cals;

import frc.robot.subsystems.motor.MotorCal;
import frc.robot.subsystems.motor.MotorCal.TypeMotor;

public class GatherCals {
    public boolean disable = true;
    
    public MotorCal topGather = new MotorCal(TypeMotor.SPARK, 14).setCurrLim(30).invert();

}
