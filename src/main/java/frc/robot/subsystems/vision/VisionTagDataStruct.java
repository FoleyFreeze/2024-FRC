package frc.robot.subsystems.vision;

import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.struct.Struct;

public class VisionTagDataStruct implements Struct<VisionTagData>{

    static final VisionTag emptyTag = new VisionTag();

    @Override
    public Class<VisionTagData> getTypeClass(){
        return VisionTagData.class;
    }

    @Override public String getTypeString(){
        return "struct:visionTagData";
    }

    @Override
    public int getSize(){
        return kSizeInt32 + kSizeBool + kSizeInt8 + kSizeFloat + VisionTag.struct.getSize()*4;
    }

    @Override
    public String getSchema(){
        return "int8 seqNum;float timestamp;bool isProcessed;int8 tagCount;VisionTag tag1;VisionTag tag2;VisionTag tag3;VisionTag tag4";
    }

    @Override
    public Struct<?>[] getNested(){
        return new Struct<?>[] {VisionTag.struct};
    }

    @Override
    public VisionTagData unpack (ByteBuffer bb){
        int seqNum = bb.getInt();
        float timestamp = bb.getFloat();
        boolean isProcessed = bb.get() > 0;
        byte tagCount = bb.get();
        VisionTag[] tags = new VisionTag[tagCount];
        for(int i=0;i<4;i++){
            if(i<tagCount){
                tags[i] = VisionTag.struct.unpack(bb);
            } else {
                VisionTag.struct.unpack(bb);
            }
        }
        return new VisionTagData(seqNum, isProcessed, timestamp, tags);
    }

    @Override
    public void pack(ByteBuffer bb, VisionTagData value) {
        bb.putInt(value.seqNum);
        bb.putFloat(value.timestamp);
        bb.put(value.isProcessed ? (byte) 1 : 0);
        bb.put(value.tagCount);
        for(int i=0;i<4;i++){
            if(i<value.tagCount){
                VisionTag.struct.pack(bb, value.tags[i]);
            } else {
                VisionTag.struct.pack(bb, emptyTag);
            }   
        }
    }
}
