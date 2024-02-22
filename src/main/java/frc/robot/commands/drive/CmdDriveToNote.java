package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class CmdDriveToNote extends Command{
    RobotContainer r;
    double lastError;

    double timer;

    public CmdDriveToNote(RobotContainer r){
        this.r = r;
        addRequirements(r.drive, r.gather);
    }

    @Override
    public void initialize(){
        lastError = 100;
        timer = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
        if(r.vision.hasNoteImage()){
            r.gather.setGatherPower(0.5, 0);
            lastError = r.drive.driveToLocation(r.vision.calcNoteLocation());
        } else {
            r.drive.swerveDrivePwr(new ChassisSpeeds(0, 0 , 0), false);
        }
    }

    @Override
    public void end(boolean interrupted){
        r.drive.swerveDrivePwr(new ChassisSpeeds(0, 0, 0), false);
        r.gather.setGatherPower(0, 0);
    }

    @Override
    public boolean isFinished(){
        double dt = Timer.getFPGATimestamp() - timer;
        boolean currentExit = dt > 0.25 && r.gather.getCurrent() > 20;
        return lastError < Units.inchesToMeters(12) || currentExit;
    }
}
