package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

    @AutoLog
    public static class VisionIOInputs{
        VisionNoteData noteData = new VisionNoteData();
        VisionTagData tagData = new VisionTagData(0);
        double now;
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}
