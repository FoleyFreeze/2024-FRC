package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

    @AutoLog
    public static class VisionIOInputs{
        double noteTimeStamp;
        VisionData noteData;
        public int noteSeqData;
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}
