package frc.robot.cals;

import frc.robot.subsystems.motor.MotorCal;
import frc.robot.subsystems.motor.MotorCal.TypeMotor;

public class ShooterCals {
    public boolean disable = false;

    public double initAngleJog = 0;
    public double initSpeedJog = 0;
    public double jogAngleIncriment = .5;
    public double jogSpeedIncriment = 200;

    public double camDistance[] = {0, 0};
    public double camAngle[] = {0, 0};
    public double camVelocity[] = {0, 0};
    public double camRPM[] = {0, 0};
 
    public double fixedAngle[] = {25, 50, 80};
    public double fixedRPM[] = {4000, 3000, 2000};
 
    public String fixedName[] = {"layup", "freethrow", "threepoint"};

    public double homePosition = 0;

    public MotorCal angleMotor = new MotorCal(TypeMotor.SPARK, 0);
    public MotorCal shootMotorTop = new MotorCal(TypeMotor.FALCON, 13);
    public MotorCal shootMotorBottom = new MotorCal(TypeMotor.FALCON, 14);

    public double allowedAngleError = 1;
    public double allowedRPMError = 100;
}    