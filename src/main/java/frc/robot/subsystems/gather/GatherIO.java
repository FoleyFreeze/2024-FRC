package frc.robot.subsystems.gather;

import org.littletonrobotics.junction.AutoLog;

public interface GatherIO {
    @AutoLog
    public static class GatherIOInputs {
        public double gatherPosition;
        public double gatherVelocity;
        public double gatherAppliedVolts;
        public double gatherCurrentAmps;       
    }

    public default void updateInputs(GatherIOInputs inputs) {}

    public default void setGatherVoltage(double voltage) {}
}
