package frc.robot.subsystems.vision;

import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.struct.Struct;

public class VisionTagDataStruct implements Struct<VisionTagData>{
    @Override
    public Class<VisionTagData> getTypeClass(){
        return VisionTagData.class;
    }

    @Override public String getTypeString(){
        return "struct:visionData";
    }

    @Override
    public int getSize(){
        return kSizeInt8 *3 + kSizeDouble + Pose3d.struct.getSize();
    }

    @Override
    public String getSchema(){
        return "int8 tagId;int8 type;int8 eBits;double decisionMargin;Pose3d pose";
    }

    @Override
    public Struct<?>[] getNested(){
        return new Struct<?>[] {Pose3d.struct};
    }

    @Override
    public VisionTagData unpack (ByteBuffer bb){
        byte tagId = bb.get();
        byte type = bb.get();
        byte eBits = bb.get();
        double d = bb.getDouble();
        Pose3d p = Pose3d.struct.unpack(bb);
        return new VisionTagData(tagId, type, eBits, d, p);
    }

    @Override
    public void pack(ByteBuffer bb, VisionTagData value) {
        bb.put(value.tagId);
        bb.put(value.type);
        bb.put(value.eBits);
        bb.putDouble(value.decisionMargin);
        Pose3d.struct.pack(bb, value.pose);
    }
}
