package frc.robot.subsystems.vision;

import java.nio.ByteBuffer;
import java.util.EnumSet;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.RawSubscriber;

public class VisionIO_HW implements VisionIO{

    private RawSubscriber poseMsgNote;
    private ByteBuffer poseDataNote;

    private BooleanEntry active;
    private BooleanEntry notesActive;

    VisionNoteData noteData = new VisionNoteData();

    
    public VisionIO_HW(){
        active = NetworkTableInstance.getDefault().getBooleanTopic("Vision/Active").getEntry(true);
        notesActive = NetworkTableInstance.getDefault().getBooleanTopic("Vision/Note Enable").getEntry(true);
        active.set(true);
        notesActive.set(true);


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
    }

    @Override
    public void updateInputs (VisionIOInputs inputs){
        inputs.noteData = noteData;

        inputs.now = Logger.getRealTimestamp()/1000000.0;

        active.set(true);
        notesActive.set(true);
    }
}
