package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

public class RoboState {
    public enum ClimbState {
        NONE,
        DEPLOYED,
        HOOKED,
        CLIMBED
    }

    RobotContainer r;

    public boolean isPrime;
    public boolean hasNote;//in shooter or arm
    public boolean hasTransfer; //in arm, probably for amp
    public boolean autoDrive;//driver does not have control of the robot
    public ClimbState climbDeploy = ClimbState.NONE; //climb deployed, but maybe not in arm yet

    public Trigger hasNoteT = new Trigger(() -> hasNote);
    public Trigger hasTransferT = new Trigger(() -> hasTransfer);
    public Trigger climbDeployT = new Trigger(() -> climbDeploy != ClimbState.NONE);

    //states
    GenericEntry hasNoteEntry;
    GenericEntry hasTransferEntry;
    GenericEntry climbDeployEntry;

    //jogs
    GenericEntry shootAngleJogEntry;
    GenericEntry shootSpeedJogEntry;
    GenericEntry shootLobSpeedJogEntry;
    GenericEntry armAngleJogEntry;

    //positions
    GenericEntry shootAngleEntry;
    GenericEntry armAngleEntry;

    //temps
    GenericEntry driveTempsEntry;
    GenericEntry swerveTempsEntry;
    GenericEntry gatherTempsEntry;
    GenericEntry shootTempsEntry;
    GenericEntry slapTempsEntry;

    public RoboState(RobotContainer r){
        this.r = r;

        //going to use state to setup the shuffleboard tab
        ShuffleboardTab teleTab = Shuffleboard.getTab("Teleop");
        hasNoteEntry = teleTab.add("Has Note", false)
            //.withWidget("Boolean Box")
            .withPosition(0,0)
            .withSize(5,4)
            .getEntry();
        
        hasTransferEntry = teleTab.add("Has Xfer", false)
            .withPosition(5, 0)
            .getEntry();
        
        climbDeployEntry = teleTab.add("Climb Deployed", false)
            .withPosition(5, 1)
            .getEntry();

        shootAngleJogEntry = teleTab.add("ShootAngleJog", 0)
            .withPosition(8, 0)
            .getEntry();

        shootSpeedJogEntry = teleTab.add("ShootSpeedJog", 0)
            .withPosition(8, 1)
            .getEntry();

        shootLobSpeedJogEntry = teleTab.add("ShootLobSpeedJog", 0)
            .withPosition(8, 2)
            .getEntry();

        armAngleJogEntry = teleTab.add("ArmAngleJog", 0)
            .withPosition(8, 3)
            .getEntry();

        shootAngleEntry = teleTab.add("ShootAngle", 0)
            .withPosition(7, 2)
            .getEntry();

        armAngleEntry = teleTab.add("ArmAngle", 0)
            .withPosition(7, 3)
            .getEntry();
        
        driveTempsEntry = teleTab.add("Drive Temps", "")
            .withPosition(7, 0)
            .getEntry();

        swerveTempsEntry = teleTab.add("Swerve Temps", "")
            .withPosition(6, 0)
            .getEntry();

        gatherTempsEntry = teleTab.add("Gather Temps", "")
            .withPosition(6, 1)
            .getEntry();
        
        shootTempsEntry = teleTab.add("Shoot Temps", "")
            .withPosition(6, 2)
            .getEntry();

        slapTempsEntry = teleTab.add("Slap Temps", "")
            .withPosition(6, 3)
            .getEntry();
    }

    public void periodic(){
        Logger.recordOutput("State/hasNote", hasNote);
        Logger.recordOutput("State/hasTransfer", hasTransfer);
        Logger.recordOutput("State/climbDeploy", climbDeploy);

        hasNoteEntry.setBoolean(hasNote);
        hasTransferEntry.setBoolean(hasTransfer);
        climbDeployEntry.setBoolean(climbDeploy != ClimbState.NONE);

        shootAngleJogEntry.setDouble(r.shooter.angleJog);
        shootSpeedJogEntry.setDouble(r.shooter.speedJog);
        shootLobSpeedJogEntry.setDouble(r.shooter.speedJogLob);
        armAngleJogEntry.setDouble(r.slappah.angleJog);

        shootAngleEntry.setDouble(r.shooter.inputs.anglePosition);
        armAngleEntry.setDouble(r.slappah.inputs.anglePosition);

        driveTempsEntry.setString(String.format("%.0f %.0f %.0f %.0f", r.drive.wheels[0].inputs.driveTemp,
                                                                              r.drive.wheels[1].inputs.driveTemp,
                                                                              r.drive.wheels[2].inputs.driveTemp,
                                                                              r.drive.wheels[3].inputs.driveTemp));
        swerveTempsEntry.setString(String.format("%.0f %.0f %.0f %.0f", r.drive.wheels[0].inputs.swerveTemp,
                                                                               r.drive.wheels[1].inputs.swerveTemp,
                                                                               r.drive.wheels[2].inputs.swerveTemp,
                                                                               r.drive.wheels[3].inputs.swerveTemp));
        gatherTempsEntry.setString(String.format("In:%.0f Gate:%.0f", r.gather.inputs.intakeTemp,
                                                                             r.gather.inputs.gateTemp));
        shootTempsEntry.setString(String.format("Ang:%.0f T:%.0f B:%.0f", r.shooter.inputs.angleTemp,
                                                                                 r.shooter.inputs.shootTopTemp,
                                                                                 r.shooter.inputs.shootBottomTemp));
        slapTempsEntry.setString(String.format("Ang:%.0f Xfer:%.0f", r.slappah.inputs.angleTemp,
                                                                            r.slappah.inputs.transferTemp));
    }
}
