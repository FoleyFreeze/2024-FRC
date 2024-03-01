package frc.robot.auton;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Locations {

    //helper distances
    private static final double fieldWidth = Units.feetToMeters(26 + 11.25/12);//feet feeeettttt
    private static final double fieldLength = Units.feetToMeters(54 + 3.25/12); 

    //all meters
    //TODO: finish this
    public static final Translation2d noteA = new Translation2d (Units.inchesToMeters(114), fieldWidth/2 + Units.inchesToMeters(57 * 2 - 57));
    public static final Translation2d noteB = new Translation2d (Units.inchesToMeters(114), fieldWidth/2 + Units.inchesToMeters(57 * 2 - 57));
    public static final Translation2d noteC = new Translation2d (Units.inchesToMeters(114), fieldWidth/2 + Units.inchesToMeters(57 * 2 - 57));
    public static final Translation2d noteD = new Translation2d (Units.inchesToMeters(fieldLength/2), fieldWidth/2 + Units.inchesToMeters(fieldWidth - 29.64));
    public static final Translation2d noteE = new Translation2d (Units.inchesToMeters(fieldLength/2), fieldWidth/2 + Units.inchesToMeters(fieldWidth - 95.64));
    public static final Translation2d noteF = new Translation2d (Units.inchesToMeters(fieldLength/2), fieldWidth/2 + Units.inchesToMeters(fieldWidth/2));
    public static final Translation2d noteG = new Translation2d (Units.inchesToMeters(fieldLength/2), fieldWidth/2 + Units.inchesToMeters(fieldWidth - 161.64));
    public static final Translation2d noteH = new Translation2d (Units.inchesToMeters(fieldLength/2), fieldWidth/2 + Units.inchesToMeters(fieldWidth - 227.64));

    public static final Translation2d notes[] = {noteA, noteB, noteC, noteD, noteE, noteF,noteG, noteH};

    public static final Translation2d podium = new Translation2d (fieldWidth/2, 120.5);
    public static final Translation2d stageLeft = new Translation2d (0,0);//opposite of l+r from drivers station
    public static final Translation2d stageRight = new Translation2d (0,0);

    public static final Translation2d shootingPositions[] = {new Translation2d(0,0),
                                                             new Translation2d(0,0),
                                                             new Translation2d(0,0),
                                                             new Translation2d(0,0),
                                                             new Translation2d(0,0),
                                                             new Translation2d(0,0),
                                                             new Translation2d(0,0),
                                                             new Translation2d(0,0)
                                                            };


    public static AprilTagFieldLayout tagLayout;
    public static Translation2d tagSpeaker;//7
    public static Translation2d tagAmp;//6, 
    public static Translation2d tagStageLeft;//16
    public static Translation2d tagStageRight;//15
    public static Translation2d tagCenterStage;//14
    {
        try{
            tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            tagSpeaker = new Translation2d(tagLayout.getTagPose(7).get().getTranslation().getX(),tagLayout.getTagPose(7).get().getTranslation().getZ());
            tagAmp = new Translation2d(tagLayout.getTagPose(6).get().getTranslation().getX(),tagLayout.getTagPose(6).get().getTranslation().getZ());
            tagStageLeft = new Translation2d(tagLayout.getTagPose(16).get().getTranslation().getX(),tagLayout.getTagPose(16).get().getTranslation().getZ());
            tagStageRight = new Translation2d(tagLayout.getTagPose(15).get().getTranslation().getX(),tagLayout.getTagPose(15).get().getTranslation().getZ());
            tagCenterStage = new Translation2d(tagLayout.getTagPose(14).get().getTranslation().getX(),tagLayout.getTagPose(14).get().getTranslation().getZ());
        }catch(Exception e){
            e.printStackTrace();
        }
    }

}
