package frc.robot.cals;

import frc.robot.subsystems.motor.MotorCal;
import frc.robot.subsystems.motor.MotorCal.TypeMotor;

public class SlappahCals {
    public boolean disable = false;

    public MotorCal angleMotor = new MotorCal(TypeMotor.SPARK, 5)
                                    .invert()
                                    .setPIDF(0.07, 0, 0.8, 0)//.05, 0, 0
                                    .setBrakeMode(true)
                                    .setPIDPwrLim(0.40, -0.25)//.25, -.25
                                    .setCurrLim(25)
                                    .setRatio(1/45.0 * 34/44.0 * 360); //something like 100deg / 20rotations = 5
                                    //fixed now: this is actually 1 / 45.0 * 34 / 44.0 * 360deg = 6.181818

    public MotorCal transferMotor = new MotorCal(TypeMotor.SPARK, 3)
                                    .setBrakeMode(true)
                                    .setCurrLim(20)
                                    //.setRampRate(0.1)
                                    .setPIDF(0.2, 0.00000, 0.0, 0)
                                    .setRatio(1 / 3.0)  //was 4.0 before 3/21/24
                                    .setPIDPwrLim(0.6);

    public double allowedAngleError = 4; //degrees
    public double allowedTransferError = 2; //rotations

    public double startAngle = 0; //degrees

   
}
