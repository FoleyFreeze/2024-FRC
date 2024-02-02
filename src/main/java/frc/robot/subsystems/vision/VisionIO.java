package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

    @AutoLog
    public static class VisionIOInputs{
        double noteTimeStamp;
        VisionData noteData;
        int noteSeqData;

        double now;
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}
