package frc.robot.subsystems.vision;

import java.nio.ByteBuffer;
import java.util.EnumSet;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.RawSubscriber;

public class VisionIO_HW implements VisionIO{

    private RawSubscriber poseMsgNote;
    private ByteBuffer poseDataNote;

    private RawSubscriber poseMsgTag;
    private ByteBuffer poseDataTag;

    private BooleanEntry active;
    private BooleanEntry notesActive;
    private BooleanEntry tagsActive;

    VisionNoteData noteData = new VisionNoteData();
    VisionTagData tagData = new VisionTagData(0);

    //based on the 2023-FRC project
    public VisionIO_HW(){
        active = NetworkTableInstance.getDefault().getBooleanTopic("/Vision/Active").getEntry(true);
        notesActive = NetworkTableInstance.getDefault().getBooleanTopic("/Vision/Note Enable").getEntry(true);
        tagsActive = NetworkTableInstance.getDefault().getBooleanTopic("/Vision/Tag Enable").getEntry(true);
        active.set(true);
        notesActive.set(true);
        tagsActive.set(true);


        poseMsgNote = NetworkTableInstance.getDefault().getTable("Vision").getRawTopic("Note Pose Data Bytes").subscribe("raw", null);
        NetworkTableInstance.getDefault().addListener(poseMsgNote,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            (event) -> {
            poseDataNote = ByteBuffer.wrap(event.valueData.value.getRaw());
            byte type = poseDataNote.get(12);
            byte numTags = poseDataNote.get(13);
            int seqNum = poseDataNote.getInt(0);
            float current = Logger.getRealTimestamp()/1000000.0f;
            float timeStamp = current - ((current - poseDataNote.getFloat(4) + poseDataNote.getFloat(8)) / 2.0f);

            VisionNoteData vd = null;
            if(numTags > 0){
                vd = new VisionNoteData(seqNum, timeStamp, poseDataNote.getFloat(18), (float)Math.toRadians(poseDataNote.getFloat(14)));
            }
            if(vd != null){
                noteData = vd;
            }
        });

        //replaced by limelight
      /*  poseMsgTag = NetworkTableInstance.getDefault().getTable("Vision").getRawTopic("Tag Pose Data Bytes").subscribe("raw", null);
        NetworkTableInstance.getDefault().addListener(poseMsgTag, 
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            event -> {
                poseDataTag = ByteBuffer.wrap(event.valueData.value.getRaw());
                byte type = poseDataTag.get(12); // type 1 = tag, type 2 = cone, type 3 = cube
                byte numTags = poseDataTag.get(13);
                if(type == 1 && numTags <= 4){//only supports 4 tags in one frame
                    VisionTagData e = new VisionTagData(numTags);
                    e.seqNum = poseDataTag.getInt(0);
                    float current = Logger.getRealTimestamp()/1000000.0f;
                    e.timestamp = current - ((current -  poseDataTag.getFloat(4) + poseDataTag.getFloat(8)) / 2.0f);
                    // added tag decision margin (1 float) and error bits (1 byte) to message
                    // this takes each tag struct from 25 bytes to 30 bytes
                    // old: for(int i = 0, b = 14; i < numTags; i++, b += 25){ 
                    for(int i = 0, b = 14; i < numTags; i++, b += 30) { 
                        
                        VisionTag visionData = new VisionTag(poseDataTag.get(b), 
                            poseDataTag.get(b+1), poseDataTag.getFloat(b+2), 
                            poseDataTag.getFloat(b+6), poseDataTag.getFloat(b+10), poseDataTag.getFloat(b+14), 
                            poseDataTag.getFloat(b+18), poseDataTag.getFloat(b+22), poseDataTag.getFloat(b+26));
                        e.tags[i] = visionData;

                    }
                    tagData = e;
                }

            });*/
    }

    @Override
    public void updateInputs (VisionIOInputs inputs, Rotation2d robotAngle) {
        inputs.noteData = noteData;
        inputs.tagData = tagData;

        inputs.now = Logger.getRealTimestamp()/1000000.0;

        active.set(true);
        notesActive.set(true);
        tagsActive.set(true);

        //limelight
        LimelightHelpers.SetRobotOrientation("limelight", robotAngle.getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        inputs.mt2_botPose = mt2.pose;
        inputs.mt2_latency = mt2.latency;
        inputs.mt2_tagCount = mt2.tagCount;
        inputs.mt2_timestamp = mt2.timestampSeconds;
        inputs.mt2_avgTagDist = mt2.avgTagDist;
    }
}
