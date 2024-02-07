package frc.robot.subsystems.vision;

import edu.wpi.first.util.struct.StructSerializable;

public class VisionNoteData implements StructSerializable{
    public int seqNum;
    public float timeStamp;
    public float distance;
    public float angle;

    public static final VisionNoteDataStruct struct = new VisionNoteDataStruct();

    public VisionNoteData(int seqNum, float timeStamp, float distance, float angle) {
        this.seqNum = seqNum;
        this.timeStamp = timeStamp;
        this.distance = distance;
        this.angle = angle;
    }

    public VisionNoteData(){}


}
