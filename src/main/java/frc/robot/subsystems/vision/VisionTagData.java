package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.util.struct.StructSerializable;

public class VisionTagData implements StructSerializable {
    public byte tagId;
    public byte type;
    public byte eBits;
    public double decisionMargin;
    public Pose3d pose;

    public VisionTagData(){
        pose = new Pose3d();
    }

    public VisionTagData(byte type, byte tagId, byte eBits, double decisionMargin, double rotX, double rotY, double rotZ, double transX, double transY, double transZ) {
        this.tagId = tagId;
        this.type = type;
        this.eBits = eBits;
        this.decisionMargin = decisionMargin;
        this.pose = new Pose3d(transX, transY, transZ, new Rotation3d(rotX, rotY, rotZ));
    }

    public VisionTagData(byte type, byte tagId, double rotX, double rotY, double rotZ, double transX, double transY, double transZ){
        this.tagId = tagId;
        this.type = type;
        this.pose = new Pose3d(transX, transY, transZ, new Rotation3d(rotX, rotY, rotZ));
    }

    public VisionTagData(byte type, double rotX, double rotY, double rotZ, double transX, double transY, double transZ){
        this.type = type;
        this.pose = new Pose3d(transX, transY, transZ, new Rotation3d(rotX, rotY, rotZ));
    }

    public VisionTagData(byte tagId, byte type, byte eBits, double decisionMargin, Pose3d pose){
        this.tagId = tagId;
        this.type = type;
        this.eBits = eBits;
        this.decisionMargin = decisionMargin;
        this.pose = pose;
    }

    public static final VisionTagDataStruct struct = new VisionTagDataStruct();
}