package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double winchRPosition;
        public double winchRVelocity;
        public double winchRAppliedVolts;
        public double winchRCurrentAmps; 
        public double winchRTemp;

        public double winchLPosition;
        public double winchLVelocity;
        public double winchLAppliedVolts;
        public double winchLCurrentAmps;
        public double winchLTemp;
    }


    public default void updateInputs(ClimberIOInputs inputs){}

    public default void setWinchVoltage(double voltageL, double voltageR){}

    public default void setWinchPosition(double positionL, double positionR){}

}
