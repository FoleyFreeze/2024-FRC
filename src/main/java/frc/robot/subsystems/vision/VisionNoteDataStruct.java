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
        return kSizeInt32 + kSizeFloat*3 + 1;
    }

    @Override
    public String getSchema() {
        return "int seqNum;float timeStamp;float distance;float angle;boolean isProcessed";
    }

    @Override
    public VisionNoteData unpack(ByteBuffer bb) {
        VisionNoteData v = new VisionNoteData();
        v.seqNum = bb.getInt();
        v.timeStamp = bb.getFloat();
        v.distance = bb.getFloat();
        v.angle = bb.getFloat();
        v.isProcessed = bb.get() > 0;
        return v;
    }

    @Override
    public void pack(ByteBuffer bb, VisionNoteData value) {
        bb.putInt(value.seqNum);
        bb.putFloat(value.timeStamp);
        bb.putFloat(value.distance);
        bb.putFloat(value.angle);
        bb.put(value.isProcessed ? (byte) 1 : (byte) 0);
    }
    
}
