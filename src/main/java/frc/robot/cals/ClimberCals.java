package frc.robot.cals;

import frc.robot.subsystems.motor.MotorCal;
import frc.robot.subsystems.motor.MotorCal.TypeMotor;

public class ClimberCals {
    public boolean disable = false;

    //twenty percent power at 10 degrees
    public double balanceKP = .2/10;

    public MotorCal winchR = new MotorCal(TypeMotor.SPARK, 6)   .invert()
                                                                .setCurrLim(40)
                                                                .setRampRate(0.1)
                                                                .setBrakeMode(false)
                                                                .setRatio(1/40.0);

    public MotorCal winchL = new MotorCal(TypeMotor.SPARK, 15)  .setCurrLim(40)
                                                                .setRampRate(0.1)
                                                                .setBrakeMode(false)
                                                                .setRatio(1/40.0);


}
