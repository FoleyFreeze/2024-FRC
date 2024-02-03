package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class CmdDriveToNote extends Command{
    RobotContainer r;
    double lastError;

    public CmdDriveToNote(RobotContainer r){
        this.r = r;
        addRequirements(r.drive, r.gather);
    }

    @Override
    public void initialize(){
        lastError = 100;
    }

    @Override
    public void execute(){
        if(r.vision.hasNoteImage()){
            r.gather.setMotorPower(0.5);
            lastError = r.drive.driveToLocation(r.vision.getNoteLocation());
        } else {
            r.drive.swerveDrivePwr(new ChassisSpeeds(0, 0 , 0), false);
        }
    }

    @Override
    public void end(boolean interrupted){
        r.drive.swerveDrivePwr(new ChassisSpeeds(0, 0 , 0), false);
    }

    @Override
    public boolean isFinished(){
        return lastError < 12 || r.gather.getCurrent() > 20;
    }
}
