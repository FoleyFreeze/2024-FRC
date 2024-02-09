package frc.robot.cals;

import frc.robot.subsystems.motor.MotorCal;
import frc.robot.subsystems.motor.MotorCal.TypeMotor;

public class ShooterCals {
    public boolean disable = true;

    public double camDistance[] = {0, 0};
    public double camAngle[] = {0, 0};
    public double camVelocity[] = {0, 0};
    public double camRPM[] = {0, 0};
 
    public double fixedAngle[] = {0, 0, 0};
    public double fixedRPM[] = {0, 0, 0};
 
    public String fixedName[] = {"layup", "freethrow", "threepoint"};

    public double homePosition = 0;

    public MotorCal angleMotor = new MotorCal(TypeMotor.SPARK, 0);
    public MotorCal shootMotorTop = new MotorCal(TypeMotor.SPARK, 0);
    public MotorCal shootMotorBottom = new MotorCal(TypeMotor.SPARK, 0);

    public double allowedAngleError = 1;
    public double allowedRPMError = 100;

}    