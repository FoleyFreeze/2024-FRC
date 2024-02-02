package frc.robot.subsystems.vision;

import java.nio.ByteBuffer;
import java.util.EnumSet;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.RawSubscriber;

public class VisionIO_HW implements VisionIO{

    private RawSubscriber poseMsgNote;
    private ByteBuffer poseDataNote;
    int noteSeqNum;
    VisionData noteData;
    double noteTimeStamp;
    
    public VisionIO_HW(){
        poseMsgNote = NetworkTableInstance.getDefault().getTable("Vision").getRawTopic("Note Pose Data Bytes").subscribe("raw", null);
        NetworkTableInstance.getDefault().addListener(poseMsgNote,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            (event) -> {
            poseDataNote = ByteBuffer.wrap(event.valueData.value.getRaw());
            byte type = poseDataNote.get(12);
            byte numTags = poseDataNote.get(13);
            int seqNum = poseDataNote.getInt(0);
            double current = Logger.getRealTimestamp()/1000000.0;
            double timestamp = current - ((current - poseDataNote.getFloat(4) + poseDataNote.getFloat(4)) / 2.0);

            VisionData vd = null;
            if(numTags > 0){
                vd = new VisionData(type, 0, Math.toRadians(poseDataNote.getFloat(28)), 0, 0, 0, poseDataNote.getFloat(14+18));
            }
            if(vd != null){
                synchronized (this){
                    noteSeqNum = seqNum;
                    noteTimeStamp = timestamp;
                    noteData = vd;
                }
            }
        });
    }



    @Override
    public void updateInputs (VisionIOInputs inputs){
        inputs.noteTimeStamp = noteTimeStamp;
        inputs.noteSeqData = noteSeqNum;
        inputs.noteData = noteData;

        inputs.now = Logger.getRealTimestamp()/1000000.0;
    }
}
