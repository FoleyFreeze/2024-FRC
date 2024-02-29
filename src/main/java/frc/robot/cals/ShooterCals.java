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

    public double startAngle = 55;
    public double homePosition = 28;

    public MotorCal angleMotor = new MotorCal(TypeMotor.SPARK, 15)
                                        .setBrakeMode(true)
                                        .setCurrLim(30)
                                        .setPIDF(2, 0, 0, 0)
                                        .setPIDPwrLim(0.4, -0.4)
                                        .setRatio(95 / 32.0 * 30/24.0); //32 rotations of the screw, but gear ratio of 30/24
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