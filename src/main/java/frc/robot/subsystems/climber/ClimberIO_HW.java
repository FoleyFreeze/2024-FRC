package frc.robot.subsystems.climber;

import frc.robot.cals.ClimberCals;
import frc.robot.subsystems.motor.Motor;

public class ClimberIO_HW  implements ClimberIO{
    
    Motor winchR;
    Motor winchL;

    public ClimberIO_HW (ClimberCals k){
        winchR = Motor.create(k.winchR);
        winchL = Motor.create(k.winchL);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs){
        inputs.winchRPosition = winchR.getPosition();
        inputs.winchRVelocity = winchR.getVelocity();
        inputs.winchRCurrentAmps = winchR.getCurrent();
        inputs.winchRAppliedVolts = winchR.getVoltage();

        inputs.winchLPosition = winchL.getPosition();
        inputs.winchLVelocity = winchL.getVelocity();
        inputs.winchLCurrentAmps = winchL.getCurrent();
        inputs.winchLAppliedVolts = winchL.getVoltage();
    }

    @Override
    public void setWinchVoltage(double voltageL, double voltageR){
        winchR.setVoltage(voltageR);
        winchL.setVoltage(voltageL);
    }
}
