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

        inputs.winchRPosition = winchL.getPosition();
        inputs.winchRVelocity = winchL.getVelocity();
        inputs.winchRCurrentAmps = winchL.getCurrent();
        inputs.winchRAppliedVolts = winchL.getVoltage();
    }

    @Override
    public void setWinchVoltage(double voltage){
        winchR.setVoltage(voltage);
        winchL.setVoltage(voltage);
    }
}
