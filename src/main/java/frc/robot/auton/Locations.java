package frc.robot.auton;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Locations {

    //helper distances
    private static final double fieldWidth = Units.feetToMeters(26 + 11.25/12);//feet feeeettttt
    private static final double fieldLength = Units.feetToMeters(54 + 3.25/12); 

    //all meters
    //TODO: finish this
    public static final Translation2d blueNoteA = new Translation2d (Units.inchesToMeters(114), fieldWidth/2 + Units.inchesToMeters(57 * 2 - 57));
    public static final Translation2d blueNoteB = new Translation2d (Units.inchesToMeters(114), fieldWidth/2 + Units.inchesToMeters(57 * 2 - 57));
    public static final Translation2d blueNoteC = new Translation2d (Units.inchesToMeters(114), fieldWidth/2 + Units.inchesToMeters(57 * 2 - 57));
    public static final Translation2d blueNoteD = new Translation2d (Units.inchesToMeters(fieldLength/2), fieldWidth/2 + Units.inchesToMeters(fieldWidth - 29.64)); //FIXME: fieldwidth is already in meters
    public static final Translation2d blueNoteE = new Translation2d (Units.inchesToMeters(fieldLength/2), fieldWidth/2 + Units.inchesToMeters(fieldWidth - 95.64));
    public static final Translation2d blueNoteF = new Translation2d (Units.inchesToMeters(fieldLength/2), fieldWidth/2 + Units.inchesToMeters(fieldWidth/2));
    public static final Translation2d blueNoteG = new Translation2d (Units.inchesToMeters(fieldLength/2), fieldWidth/2 + Units.inchesToMeters(fieldWidth - 161.64));
    public static final Translation2d blueNoteH = new Translation2d (Units.inchesToMeters(fieldLength/2), fieldWidth/2 + Units.inchesToMeters(fieldWidth - 227.64));

    public static final Translation2d redNoteA = new Translation2d(0, 0);
    public static final Translation2d redNoteB = new Translation2d(0, 0);
    public static final Translation2d redNoteC = new Translation2d(0, 0);
    public static final Translation2d redNoteD = new Translation2d(0, 0);
    public static final Translation2d redNoteE = new Translation2d(0, 0);
    public static final Translation2d redNoteF = new Translation2d(0, 0);
    public static final Translation2d redNoteG = new Translation2d(0, 0);
    public static final Translation2d redNoteH = new Translation2d(0, 0);

    public static final Translation2d bluePodium = new Translation2d (fieldWidth/2, 120.5);
    public static final Translation2d blueStageLeft = new Translation2d (0,0);//opposite of l+r from drivers station
    public static final Translation2d blueStageRight = new Translation2d (0,0);

    public static final Translation2d redPodium = new Translation2d (fieldWidth/2, 120.5);
    public static final Translation2d redStageLeft = new Translation2d (0,0);//opposite of l+r from drivers station
    public static final Translation2d redStageRight = new Translation2d (0,0);

    public static final Translation2d blueShootingPositions[] = {new Translation2d(0,0),
                                                                 new Translation2d(0,0),
                                                                 new Translation2d(0,0),
                                                                 new Translation2d(0,0),
                                                                 new Translation2d(0,0),
                                                                 new Translation2d(0,0),
                                                                 new Translation2d(0,0),
                                                                 new Translation2d(0,0)
                                                                };

    public static final Translation2d redShootingPositions[] =  {new Translation2d(0,0),
                                                                 new Translation2d(0,0),
                                                                 new Translation2d(0,0),
                                                                 new Translation2d(0,0),
                                                                 new Translation2d(0,0),
                                                                 new Translation2d(0,0),
                                                                 new Translation2d(0,0),
                                                                 new Translation2d(0,0)
                                                                };

    public static final Translation2d blueNotes[] = {blueNoteA, blueNoteB, blueNoteC, blueNoteD, blueNoteE, blueNoteF, blueNoteG, blueNoteH};
    public static final Translation2d redNotes[] = {redNoteA, redNoteB, redNoteC, redNoteD, redNoteE, redNoteF, redNoteG, redNoteH};
    
    public static Translation2d[] notes = blueNotes;
    public static Translation2d[] shootingPositions = blueShootingPositions;

    public static AprilTagFieldLayout tagLayout;
    public static Translation2d tagSpeaker;//7
    public static Translation2d tagAmp;//6, 
    public static Translation2d tagStageLeft;//16
    public static Translation2d tagStageRight;//15
    public static Translation2d tagCenterStage;//14
    {
        try{
            tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            tagSpeaker = from3dTag(7);
            tagAmp = from3dTag(6);
            tagStageLeft = from3dTag(16);
            tagStageRight = from3dTag(15);
            tagCenterStage = from3dTag(14);
        }catch(Exception e){
            e.printStackTrace();
        }
    }

    private static Translation2d from3dTag(int tag){
        Translation3d tag3d = tagLayout.getTagPose(tag).get().getTranslation();
        return new Translation2d(tag3d.getX(), tag3d.getZ());
    }

    public static void recalcForAlliance(){
        if(DriverStation.getAlliance().isPresent()){
            if(DriverStation.getAlliance().get() == Alliance.Blue){
                tagSpeaker = from3dTag(7);
                tagAmp = from3dTag(6);
                tagStageLeft = from3dTag(16);
                tagStageRight = from3dTag(15);
                tagCenterStage = from3dTag(14);

                notes = blueNotes;
                shootingPositions = blueShootingPositions;
            } else {
                //FIXME: need red ids
                tagSpeaker = from3dTag(0);
                tagAmp = from3dTag(0);
                tagStageLeft = from3dTag(0);
                tagStageRight = from3dTag(0);
                tagCenterStage = from3dTag(0);

                notes = redNotes;
                shootingPositions = redShootingPositions;
            }
        }

        
    }

}
