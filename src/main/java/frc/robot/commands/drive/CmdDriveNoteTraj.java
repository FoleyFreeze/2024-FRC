package frc.robot.commands.drive;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class CmdDriveNoteTraj extends Command{

    RobotContainer r;

    double maxVelocity;
    double maxAccel;

    public CmdDriveNoteTraj(RobotContainer r){
        this.r = r;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

    }

    @Override 
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }

    public Trajectory generateTrajectory(Pose2d end){
        TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAccel);
        config.setStartVelocity(r.drive.getVelocity().getNorm());
        
        return TrajectoryGenerator.generateTrajectory(
            r.drive.getPose(),
            new ArrayList<>(),
            end,
            config
        ); 
    }
}
