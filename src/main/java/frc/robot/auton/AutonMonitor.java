package frc.robot.auton;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutonMonitor extends Command{
    
    RobotContainer r;

    double maxVelocity = 1;//m/s
    double maxAccel = 1;//m/s/s
    double maxAngularAccel = 1;//rad/s
    double maxAngularVelocity = 0.25;//rad/s/s
    PathConstraints pathConstraints = new PathConstraints(maxVelocity, maxAccel, maxAngularVelocity, maxAngularAccel);


    ArrayList<Command> pathCommands;

    public AutonMonitor(RobotContainer r){
        this.r = r;

        pathCommands = new ArrayList<>();

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


    public Command createPathFollower(Translation2d start, Translation2d end){
        Translation2d pathVector = end.minus(start);
        
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            new Pose2d(start, pathVector.getAngle()),
            new Pose2d(end, pathVector.getAngle())
        );

        System.out.println("Created path starting at: " + bezierPoints.get(0).toString());
        System.out.println("and Ending at: " + bezierPoints.get(bezierPoints.size()-1).toString());

        PathPlannerPath path = new PathPlannerPath(
            bezierPoints, 
            pathConstraints, 
            new GoalEndState(0, r.drive.getAngle())
        );

        return new FollowPathHolonomic(
            path, 
            r.drive::getPose, 
            r.drive::getRelVelocity, 
            r.drive::swerveDrivePwr, 
            r.drive.k.notePathFollowerConfig, 
            () -> false, 
            r.drive
        );
    }

}
