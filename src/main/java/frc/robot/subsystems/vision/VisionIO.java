package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO {

    @AutoLog
    public static class VisionIOInputs{
        VisionNoteData noteData = new VisionNoteData();
        VisionTagData tagData = new VisionTagData(0);
        double now;

        //Limelight data
        Pose2d mt2_botPose = new Pose2d();
        double mt2_timestamp;
        double mt2_latency;
        int mt2_tagCount;
        double mt2_avgTagDist;
    }

    public default void updateInputs(VisionIOInputs inputs, Rotation2d botAngle) {}
}
