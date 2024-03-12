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
        int errorCount = winchR.getErrorCount();
        double position = winchR.getPosition();
        double velocity = winchR.getVelocity();
        double current = winchR.getCurrent();
        double voltage = winchR.getVoltage();
        double temp = winchR.getTemp();

        if(winchR.getErrorCount() == errorCount){
            inputs.winchRPosition = position;
            inputs.winchRVelocity = velocity;
            inputs.winchRCurrentAmps = current;
            inputs.winchRAppliedVolts = voltage;
            inputs.winchRTemp = temp;
        }

        errorCount = winchL.getErrorCount();
        position = winchL.getPosition();
        velocity = winchL.getVelocity();
        current = winchL.getCurrent();
        voltage = winchL.getVoltage();
        temp = winchL.getTemp();

        if(winchL.getErrorCount() == errorCount){
            inputs.winchLPosition = position;
            inputs.winchLVelocity = velocity;
            inputs.winchLCurrentAmps = current;
            inputs.winchLAppliedVolts = voltage;
            inputs.winchLTemp = temp;
        }
    }

    @Override
    public void setWinchVoltage(double voltageL, double voltageR){
        winchR.setVoltage(voltageR);
        winchL.setVoltage(voltageL);
    }

    @Override
    public void setWinchPosition(double positionL, double positionR){
        winchL.setPosition(positionL);
        winchR.setPosition(positionR);
    }

    @Override
    public void setBrakes(boolean on){
        winchL.setBrakeMode(on);
        winchR.setBrakeMode(on);
    }
}
