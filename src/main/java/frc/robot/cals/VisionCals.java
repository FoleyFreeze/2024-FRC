package frc.robot.cals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.auton.Locations;

public class VisionCals {
    
    public boolean disable = false;

    public double maxNoteAge = 0.2; //seconds
    public int bufferSize = (int) (maxNoteAge / 0.02) + 1;

    public double maxTagAge = 0.2;

    public Pose2d camLocation = new Pose2d(Locations.robotLength/2, 0, Rotation2d.fromDegrees(0)); 
                                    //originally 13in, but we are now dist from bumper so botlen/2 it is
                                    
    public double maxNoteDist = Units.feetToMeters(10);

    public Pose3d tagCamLocation = new Pose3d(new Translation3d(
                                                            Units.inchesToMeters(-13.25),
                                                            Units.inchesToMeters(9.5),
                                                            Units.inchesToMeters(15.0)//moved lower
                                                            ),
                                                   new Rotation3d(
                                                            0,
                                                            Math.toRadians(-26),
                                                            Math.PI
                                                            )
                                                    );

    public boolean printDebugTagData = false;

    public double[] distAxis = {Units.feetToMeters(5), Units.feetToMeters(15)};
    public double[] stdDevs = {0.7, 2};
}
