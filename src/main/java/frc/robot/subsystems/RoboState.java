package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RoboState {
    public boolean hasNote;
    public boolean hasTransfer;
    public boolean ampDeploy;
    public boolean climbDeploy;
    public boolean isPrime;

    public Trigger hasNoteT = new Trigger(() -> hasNote);
    public Trigger hasTransferT = new Trigger(() -> hasTransfer);
    public Trigger armDeployT = new Trigger(() -> ampDeploy);
    public Trigger climbDeployT = new Trigger(() -> climbDeploy);
    public Trigger isPrimeT = new Trigger(() -> isPrime);
}
