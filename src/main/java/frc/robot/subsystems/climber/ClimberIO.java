package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double winchRPosition;
        public double winchRVelocity;
        public double winchRAppliedVolts;
        public double winchRCurrentAmps; 

        public double winchLPosition;
        public double winchLVelocity;
        public double winchLAppliedVolts;
        public double winchLCurrentAmps; 
    }


    public default void updateInputs(ClimberIOInputs inputs){}

    public default void setWinchVoltage(double voltageL, double voltageR){}

}
