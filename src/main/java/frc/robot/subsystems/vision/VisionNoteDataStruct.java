package frc.robot.subsystems.vision;

import java.nio.ByteBuffer;

import edu.wpi.first.util.struct.Struct;

public class VisionNoteDataStruct implements Struct<VisionNoteData>{

    @Override
    public Class<VisionNoteData> getTypeClass() {
        return VisionNoteData.class;
    }

    @Override
    public String getTypeString() {
        return "struct:VisionNoteData";
    }

    @Override
    public int getSize() {
        return kSizeInt32 + kSizeFloat *3;
    }

    @Override
    public String getSchema() {
        return "int seqNum;float timeStamp;float distance;float angle";
    }

    @Override
    public VisionNoteData unpack(ByteBuffer bb) {
        VisionNoteData v = new VisionNoteData();
        v.seqNum = bb.getInt();
        v.timeStamp = bb.getFloat();
        v.distance = bb.getFloat();
        v.angle = bb.getFloat();
        return v;
    }

    @Override
    public void pack(ByteBuffer bb, VisionNoteData value) {
        bb.putInt(value.seqNum);
        bb.putFloat(value.timeStamp);
        bb.putFloat(value.distance);
        bb.putFloat(value.angle);
    }
    
}
