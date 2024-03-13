package frc.robot.cals;

import frc.robot.subsystems.motor.MotorCal;
import frc.robot.subsystems.motor.MotorCal.TypeMotor;

public class ClimberCals {
    public boolean disable = false;

    public double climbUpPwr = .25;
    public double climbDownPwr = -.05;
    public double jogWinchAmount = 1/16.0;

    //15 percent power at 10 degrees
    public double balanceKP = .15/10;

    public MotorCal winchR = new MotorCal(TypeMotor.SPARK, 6)   .setCurrLim(40)
                                                                .setRampRate(0.1)
                                                                .setBrakeMode(true)
                                                                .setPIDF(0.04, 0, 0, 0)
                                                                .setPIDPwrLim(0.4)
                                                                .setRatio(1/40.0);

    public MotorCal winchL = new MotorCal(TypeMotor.SPARK, 15)  .invert()
                                                                .setCurrLim(40)
                                                                .setRampRate(0.1)
                                                                .setBrakeMode(true)
                                                                .setPIDF(0.04, 0, 0, 0)
                                                                .setPIDPwrLim(0.4)
                                                                .setRatio(1/40.0);


}
