package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class CmdDrive extends Command {
    
    RobotContainer r;

    public CmdDrive(RobotContainer r){
        this.r = r;
        addRequirements(r.drive);
    }
    
    @Override
    public void execute(){
        r.drive.swerveDrivePwr(r.inputs.getChassisSpeeds(), false);    
    }

    @Override
    public void end(boolean interrupted){
        r.drive.swerveDrivePwr(new ChassisSpeeds(0, 0 , 0), false);
    }
}
