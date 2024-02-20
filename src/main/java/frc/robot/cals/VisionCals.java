package frc.robot.cals;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class VisionCals {
    
    public boolean disable = false;

    public double maxNoteAge = 0.2; //seconds
    public int bufferSize = (int) (maxNoteAge / 0.02) + 1;

    public AprilTagFieldLayout tagLayout;
    public Translation2d shootTarget;
    {
        try{
            tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            shootTarget = new Translation2d(tagLayout.getTagPose(7).get().getTranslation().getX(),tagLayout.getTagPose(7).get().getTranslation().getZ());
        }catch(Exception e){
            e.printStackTrace();
        }
    }

    public Pose2d camLocation = new Pose2d(Units.inchesToMeters(12), 0, Rotation2d.fromDegrees(0)); 
}
