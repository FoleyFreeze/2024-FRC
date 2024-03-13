package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RoboState {
    public enum ClimbState {
        NONE,
        DEPLOYED,
        HOOKED,
        CLIMBED
    }
    public boolean hasNote;//in shooter or arm
    public boolean hasTransfer; //in arm, probably for amp
    public ClimbState climbDeploy = ClimbState.NONE; //climb deployed, but maybe not in arm yet


    public Trigger hasNoteT = new Trigger(() -> hasNote);
    public Trigger hasTransferT = new Trigger(() -> hasTransfer);
    public Trigger climbDeployT = new Trigger(() -> climbDeploy != ClimbState.NONE);
}
