package frc.robot.cals;

public class VisionCals {
    
    public boolean disable = true;

    public double maxNoteAge = 0.2; //seconds
    public int bufferSize = (int) (maxNoteAge / 0.02) + 1;
}
