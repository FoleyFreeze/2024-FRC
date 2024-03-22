package frc.robot.cals;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionCals {
    
    public boolean disable = false;

    public double maxNoteAge = 0.2; //seconds
    public int bufferSize = (int) (maxNoteAge / 0.02) + 1;

    public double maxTagAge = 0.2;

    public Pose2d camLocation = new Pose2d(Units.inchesToMeters(13 ), 0, Rotation2d.fromDegrees(0)); 

    public Pose3d tagCamLocation = new Pose3d(new Translation3d(
                                                            Units.inchesToMeters(-13.25),
                                                            Units.inchesToMeters(9.5),
                                                            Units.inchesToMeters(18.5)
                                                            ),
                                                   new Rotation3d(
                                                            0,
                                                            Math.toRadians(-28),
                                                            Math.PI
                                                            )
                                                    );
}
