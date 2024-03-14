package frc.robot.subsystems.vision;

public class VisionTag {
    public byte tagId;
    public byte eBits;
    public float decisionMargin;
    //public Pose3d pose; use floats for smaller log sizes
    public float transX, transY, transZ;
    public float rotX, rotY, rotZ;
    
    public VisionTag(){
        
    }

    public VisionTag(byte tagId, byte eBits, float decisionMargin, float rotX, float rotY, float rotZ, float transX, float transY, float transZ){
        this.tagId = tagId;
        this.eBits = eBits;
        this.decisionMargin = decisionMargin;
        this.transX = transX;
        this.transY = transY;
        this.transZ = transZ;
        this.rotX = rotX;
        this.rotY = rotY;
        this.rotZ = rotZ;
    }
    
    public static final VisionTagStruct struct = new VisionTagStruct();
}
