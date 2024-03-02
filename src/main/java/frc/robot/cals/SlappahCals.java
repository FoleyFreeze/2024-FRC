package frc.robot.cals;

import frc.robot.subsystems.motor.MotorCal;
import frc.robot.subsystems.motor.MotorCal.TypeMotor;

public class SlappahCals {
    public boolean disable = false;

    public MotorCal angleMotor = new MotorCal(TypeMotor.SPARK, 5)
                                    .invert()
                                    .setPIDF(0.05, 0, 0.8, 0)
                                    .setBrakeMode(true)
                                    .setPIDPwrLim(0.25, -0.25)
                                    .setCurrLim(15)
                                    .setRatio(100 / 20.0); //something like 100deg / 20rotations

    public MotorCal transferMotor = new MotorCal(TypeMotor.SPARK, 3)
                                    .invert()
                                    .setBrakeMode(true)
                                    .setCurrLim(20)
                                    .setRampRate(0.1)
                                    .setPIDF(0.1, 0.001, 0, 0)
                                    .setRatio(1 / 4.0)
                                    .setPIDPwrLim(0.6);

    public double allowedAngleError = 4; //degrees
    public double allowedTransferError = 0.25; //rotations

    public double startAngle = 0; //degrees

   
}
