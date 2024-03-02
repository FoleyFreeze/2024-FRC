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
 
    public double maxFixedAngle = 61;
    public double minFixedAngle = 22;
    public double maxFixedSpeed = 10000;
    public double minFixedSpeed = 500;
    public double fixedAngle[] = {30, 50, 60};
    public double fixedRPM[] = {7000, 6000, 5000};
 
    public String fixedName[] = {"layup", "freethrow", "threepoint"};

    public double startAngle = 55;
    public double homePosition = 35;

    public MotorCal angleMotor = new MotorCal(TypeMotor.SPARK, 16)
                                        .setBrakeMode(true)
                                        .setCurrLim(30)
                                        .setPIDF(2, 0, 0, 0)
                                        .setPIDPwrLim(0.3, -0.3)//TODO: go back to 0.4
                                        .setRatio(85 / 32.0 * 30/24.0); //32 rotations of the screw, but gear ratio of 30/24
                                                                        //so a real ratio of 95deg / 25.6 (yes its geared up)

    public MotorCal shootMotorTop = new MotorCal(TypeMotor.FALCON, 13)
                                        .setPIDF(0.5, 0, 0, 0.1177)
                                        .setBrakeMode(false)
                                        .invert()
                                        .setCurrLim(40)
                                        .setPIDPwrLim(1, 0)
                                        .setRampRate(0.05)
                                        .setRatio(42 / 24.0); //geared up 42:24

    public MotorCal shootMotorBottom = new MotorCal(TypeMotor.FALCON, 14)
                                        .setPIDF(0.5, 0, 0, 0.1177)
                                        .setBrakeMode(false)
                                        .invert()
                                        .setCurrLim(40)
                                        .setPIDPwrLim(1, 0)
                                        .setRampRate(0.05)
                                        .setRatio(42 / 24.0); //geared up 42:24;

    public double allowedAngleError = 1;
    public double allowedRPMError = 100;
}    