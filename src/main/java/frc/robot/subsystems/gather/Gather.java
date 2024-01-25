package frc.robot.subsystems.gather;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.cals.GatherCals;
import frc.robot.subsystems.motor.Motor;

public class Gather extends SubsystemBase{
    
    GatherCals k;
    RobotContainer r;
    Motor topGather;

    public Gather (GatherCals k, RobotContainer r){
        this.k = k;
        this.r = r;

        topGather = Motor.create(k.topGather);
    }

    public void setMotorPower(double power){
        topGather.setVoltage(power*12);
    }
}
