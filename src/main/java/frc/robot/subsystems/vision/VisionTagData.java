package frc.robot.subsystems.vision;

import edu.wpi.first.util.struct.StructSerializable;

public class VisionTagData implements StructSerializable {
    public int seqNum;
    public float timestamp;
    public boolean isProcessed;
    public byte tagCount;
    public VisionTag[] tags;

    public VisionTagData(int tagCount){
        this.tagCount = (byte) tagCount;
        tags = new VisionTag[tagCount];
    }

    public VisionTagData(int seqNum, boolean isProcessed, float timestamp, VisionTag[] tags){
        this.seqNum = seqNum;
        this.isProcessed = isProcessed;
        this.timestamp = timestamp;
        tagCount = (byte) tags.length;
        this.tags = tags;
    }

    public static final VisionTagDataStruct struct = new VisionTagDataStruct();
}