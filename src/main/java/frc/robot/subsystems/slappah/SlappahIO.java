package frc.robot.subsystems.slappah;

import org.littletonrobotics.junction.AutoLog;

public interface SlappahIO {
    @AutoLog
    public static class SlappahIOInputs{
        public double transferPosition;
        public double transferVelocity;
        public double transferAppliedVolts;
        public double transferCurrentAmps;    

        public double anglePosition;
        public double angleVelocity;
        public double angleAppliedVolts;
        public double angleCurrentAmps;  
        
    }

    public default void updateInputs(SlappahIOInputs inputs){}

    public default void setPosition(double angle){}

    public default void setAngleVoltage(double voltage){}

    public default void setArmEncoderPosition(double angle){}

    public default void setTransferVoltage(double voltage){}

    public default void setTransferPosition(double position){}
}
