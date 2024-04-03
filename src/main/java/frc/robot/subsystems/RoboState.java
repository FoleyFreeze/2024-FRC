package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RoboState {
    public enum ClimbState {
        NONE,
        DEPLOYED,
        HOOKED,
        CLIMBED
    }

    public boolean isPrime;
    public boolean hasNote;//in shooter or arm
    public boolean hasTransfer; //in arm, probably for amp
    public boolean autoDrive;//driver does not have control of the robot
    public ClimbState climbDeploy = ClimbState.NONE; //climb deployed, but maybe not in arm yet


    public Trigger hasNoteT = new Trigger(() -> hasNote);
    public Trigger hasTransferT = new Trigger(() -> hasTransfer);
    public Trigger climbDeployT = new Trigger(() -> climbDeploy != ClimbState.NONE);

    GenericEntry hasNoteEntry;

    public RoboState(){
        ShuffleboardTab teleTab = Shuffleboard.getTab("Teleop");
        hasNoteEntry = teleTab.add("Has Note", false)
            //.withWidget("Boolean Box")
            .withPosition(0,0)
            .withSize(9,4)
            .getEntry();
    }

    public void periodic(){
        Logger.recordOutput("State/hasNote", hasNote);
        Logger.recordOutput("State/hasTransfer", hasTransfer);
        Logger.recordOutput("State/climbDeploy", climbDeploy);

        hasNoteEntry.setBoolean(hasNote);
    }
}
