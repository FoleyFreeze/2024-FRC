package frc.robot.auton;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Locations {
    //all meters

    //helper distances
    public static final double robotWidth = Units.inchesToMeters(32.75);
    public static final double robotLength = Units.inchesToMeters(38.25);

    private static final double fieldWidth = Units.feetToMeters(26 + 11.25/12);//feet feeeettttt
    private static final double fieldLength = Units.feetToMeters(54 + 3.25/12);
    private static final double blueWingLine = Units.inchesToMeters(76.1+155.1);
    private static final double redWingLine = fieldLength - Units.inchesToMeters(76.1+155.1);

    private static final double blueSpeakerEdge = Units.inchesToMeters(76.1 - 39.93);
    private static final double redSpeakerEdge = fieldLength - Units.inchesToMeters(76.1 - 39.93);
    private static final double speakerEdgeLength = Units.inchesToMeters(41);

    //notes
    public static final Translation2d blueNoteA = new Translation2d (Units.inchesToMeters(114), fieldWidth/2 + Units.inchesToMeters(57 * 2));
    public static final Translation2d blueNoteB = new Translation2d (Units.inchesToMeters(114), fieldWidth/2 + Units.inchesToMeters(57));
    public static final Translation2d blueNoteC = new Translation2d (Units.inchesToMeters(114), fieldWidth/2);
    public static final Translation2d blueNoteD = new Translation2d (Units.inchesToMeters(fieldLength/2), fieldWidth - Units.inchesToMeters(29.625));
    public static final Translation2d blueNoteE = new Translation2d (Units.inchesToMeters(fieldLength/2), fieldWidth - Units.inchesToMeters(66 + 29.625));
    public static final Translation2d blueNoteF = new Translation2d (Units.inchesToMeters(fieldLength/2), fieldWidth/2);
    public static final Translation2d blueNoteG = new Translation2d (Units.inchesToMeters(fieldLength/2), fieldWidth/2 + Units.inchesToMeters(66));
    public static final Translation2d blueNoteH = new Translation2d (Units.inchesToMeters(fieldLength/2), fieldWidth/2 + Units.inchesToMeters(66*2));

    public static final Translation2d redNoteA = new Translation2d(fieldLength - Units.inchesToMeters(144), fieldWidth/2);
    public static final Translation2d redNoteB = new Translation2d(fieldLength - Units.inchesToMeters(144), fieldWidth/2 + Units.inchesToMeters(57));
    public static final Translation2d redNoteC = new Translation2d(fieldLength - Units.inchesToMeters(144), fieldWidth/2 + Units.inchesToMeters(57*2));
    public static final Translation2d redNoteD = blueNoteH;//same locations, reverse order
    public static final Translation2d redNoteE = blueNoteG;
    public static final Translation2d redNoteF = blueNoteF;
    public static final Translation2d redNoteG = blueNoteE;
    public static final Translation2d redNoteH = blueNoteD;

    //start locations
    public static final Pose2d blueSpeakerSide = new Pose2d(new Translation2d(robotLength/2, robotWidth/2).rotateBy(Rotation2d.fromDegrees(60)).plus(new Translation2d(blueSpeakerEdge, blueNoteB.getY()+speakerEdgeLength/2.0)), Rotation2d.fromDegrees(60));
    public static final Pose2d blueCenterSide = new Pose2d(new Translation2d(blueSpeakerEdge + robotLength/2.0, blueNoteB.getY()), new Rotation2d());
    public static final Pose2d blueSourceSide = new Pose2d(new Translation2d(robotLength/2, robotWidth/2).rotateBy(Rotation2d.fromDegrees(-60)).plus(new Translation2d(blueSpeakerEdge, blueNoteB.getY()-speakerEdgeLength/2.0)), Rotation2d.fromDegrees(-60));

    public static final Pose2d redSpeakerSide = new Pose2d(new Translation2d(robotLength/2, robotWidth/2).rotateBy(Rotation2d.fromDegrees(120)).plus(new Translation2d(redSpeakerEdge, redNoteB.getY()+speakerEdgeLength/2.0)), Rotation2d.fromDegrees(120));
    public static final Pose2d redCenterSide = new Pose2d(new Translation2d(redSpeakerEdge - robotLength/2.0, redNoteB.getY()), Rotation2d.fromDegrees(180));
    public static final Pose2d redSourceSide = new Pose2d(new Translation2d(robotLength/2, robotWidth/2).rotateBy(Rotation2d.fromDegrees(240)).plus(new Translation2d(redSpeakerEdge, redNoteB.getY()-speakerEdgeLength/2.0)), Rotation2d.fromDegrees(240));

    //field things
    public static final Translation2d bluePodium = new Translation2d (Units.inchesToMeters(120.5), fieldWidth/2);
    public static final Translation2d blueStageLeft = new Translation2d (0,0);//opposite of l+r from drivers station
    public static final Translation2d blueStageRight = new Translation2d (0,0);

    public static final Translation2d redPodium = new Translation2d (fieldLength - Units.inchesToMeters(120.5), fieldWidth/2);
    public static final Translation2d redStageLeft = new Translation2d (0,0);//opposite of l+r from drivers station
    public static final Translation2d redStageRight = new Translation2d (0,0);

    //shot locations
    public static final Translation2d blueShootingPositions[] = {new Translation2d(blueNoteA.getX() - Units.inchesToMeters(24), midY(blueNoteA, blueNoteB)), //between A and B, offset 2ft in
                                                                 new Translation2d(blueNoteB.getX() - Units.inchesToMeters(24), midY(blueNoteB, blueNoteC)), //between B and C, offset 2ft in
                                                                 new Translation2d(blueNoteB.getX() + Units.inchesToMeters(24), midY(blueNoteA, blueNoteB)), //between A and B, offset the other way
                                                                 new Translation2d(blueWingLine - Units.inchesToMeters(48), midY(blueNoteD, blueNoteE)), //speakerside, between DE | offset X 4ft in
                                                                 new Translation2d(blueWingLine - Units.inchesToMeters(48), fieldWidth + Units.inchesToMeters(57/2.0)), //under the stage, speaker side
                                                                 new Translation2d(blueNoteC.getX() + Units.inchesToMeters(24), blueNoteC.getY() - Units.inchesToMeters(48)), //nonspeakerside, far forward, offset from noteC
                                                                };

    public static final Translation2d redShootingPositions[] =  {new Translation2d(redNoteA.getX() + Units.inchesToMeters(24), midY(redNoteA, redNoteB)), //between A and B, offset 2ft in
                                                                 new Translation2d(redNoteB.getX() + Units.inchesToMeters(24), midY(redNoteB, redNoteC)), //between B and C, offset 2ft in
                                                                 new Translation2d(redNoteB.getX() - Units.inchesToMeters(24), midY(redNoteB, redNoteC)), //between B and C, offset the other way
                                                                 new Translation2d(redWingLine + Units.inchesToMeters(48), midY(redNoteG, redNoteH)), //speakerside, between DE | offset X 4ft in
                                                                 new Translation2d(redWingLine + Units.inchesToMeters(48), fieldWidth + Units.inchesToMeters(57/2.0)), //under the stage, speaker side
                                                                 new Translation2d(redNoteA.getX() - Units.inchesToMeters(24), redNoteA.getY() - Units.inchesToMeters(48)), //nonspeakerside, far forward, offset from noteC
                                                                };

    //things that flip on alliance change
    public static final Translation2d blueNotes[] = {blueNoteA, blueNoteB, blueNoteC, blueNoteD, blueNoteE, blueNoteF, blueNoteG, blueNoteH};
    public static final Translation2d redNotes[] = {redNoteA, redNoteB, redNoteC, redNoteD, redNoteE, redNoteF, redNoteG, redNoteH};
    public static final Pose2d[] blueStarts = {blueSpeakerSide, blueCenterSide, blueSourceSide};
    public static final Pose2d[] redStarts = {redSpeakerSide, redCenterSide, redSourceSide};
    
    public static Translation2d[] notes = blueNotes;
    public static Translation2d[] shootingPositions = blueShootingPositions;
    public static Pose2d[] startLocations = blueStarts;

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

    private static double midY(Translation2d one, Translation2d two){
        return (one.getY() + two.getY()) / 2.0;
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
                startLocations = blueStarts;
            } else {
                tagSpeaker = from3dTag(4);
                tagAmp = from3dTag(5);
                tagStageLeft = from3dTag(12);
                tagStageRight = from3dTag(11);
                tagCenterStage = from3dTag(13);

                notes = redNotes;
                shootingPositions = redShootingPositions;
                startLocations = redStarts;
            }
        }

        
    }

}
