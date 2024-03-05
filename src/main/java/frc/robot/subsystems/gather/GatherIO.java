package frc.robot.subsystems.gather;

import org.littletonrobotics.junction.AutoLog;

public interface GatherIO {
    @AutoLog
    public static class GatherIOInputs {
        public double intakePosition;
        public double intakeVelocity;
        public double intakeAppliedVolts;
        public double intakeCurrentAmps;    
        public double intakeTemp;

        public double gatePosition;
        public double gateVelocity;
        public double gateAppliedVolts;
        public double gateCurrentAmps;
        public double gateTemp;
    }

    public default void updateInputs(GatherIOInputs inputs) {}

    public default void setIntakeVoltage(double voltage) {}

    public default void setGateVoltage(double gateVoltage) {}

    public default void setGatePosition(double position) {}

}
