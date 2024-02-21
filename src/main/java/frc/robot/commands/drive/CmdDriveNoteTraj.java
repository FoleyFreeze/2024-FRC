package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class CmdDriveNoteTraj extends Command{

    RobotContainer r;

    double maxVelocity = 0.5;//m/s
    double maxAccel = 1;//m/s/s
    double maxAngularAccel = 1;//rad/s
    double maxAngularVelocity = 0.25;//rad/s/s

    PathConstraints pathConstraints = new PathConstraints(maxVelocity, maxAccel, maxAngularVelocity, maxAngularAccel);

    double singleStepThresh = Units.inchesToMeters(8);
    double totalThresh = Units.inchesToMeters(18);

    Command driveCommand;

    Translation2d robotOffset;
    Translation2d pathEndLocation;

    public CmdDriveNoteTraj(RobotContainer r){
        this.r = r;
        addRequirements(r.drive);
    }

    @Override
    public void initialize(){
        if(r.vision.hasNoteImage()){
            driveCommand = createPathFollower();
            driveCommand.initialize();
        }
    }

    @Override
    public void execute(){
        //if there is new note data
        //  calculate the new path offset so it can be applied to the robot pose
        if(r.vision.hasNoteImage()){
            if(driveCommand == null){
                driveCommand = createPathFollower();
                driveCommand.initialize();
            } else {
                Translation2d prevOffset = robotOffset;

                robotOffset = pathEndLocation.minus(r.vision.getNoteLocation());
                Logger.recordOutput("Vision/RobotOffset", robotOffset);
                Translation2d delta = robotOffset.minus(prevOffset);

                //if path has diverged too far, make a new one
                if(delta.getNorm() > singleStepThresh || robotOffset.getNorm() > totalThresh) {
                    System.out.println("Path diverged, creating new one");
                    
                    driveCommand.end(true); //call this because its doing logging things we dont understand
                                            //and we would rather not break anything
                    driveCommand = createPathFollower();
                    driveCommand.initialize();
                }
            }
        }
        
        if(driveCommand != null) driveCommand.execute();
    }

    @Override 
    public void end(boolean interrupted){
        if(driveCommand != null) driveCommand.end(interrupted);
    }

    @Override
    public boolean isFinished(){
        if(driveCommand != null){
            return driveCommand.isFinished();
        } else {
            return false;
        }
    }

    public Command createPathFollower(){
        //no offset because this path is new
        robotOffset = new Translation2d();
        pathEndLocation = r.vision.getNoteLocation();
        
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            new Pose2d(r.drive.getPose().getTranslation(), getVelocityAngle(r.drive.getVelocity())),
            new Pose2d(pathEndLocation, r.drive.getAngle())
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
            this::getRelativePose, 
            r.drive::getVelocity, 
            r.drive::swerveDrivePwr, 
            r.drive.k.notePathFollowerConfig, 
            () -> false, 
            r.drive
        );
    }

    private Rotation2d getVelocityAngle(ChassisSpeeds speed){
        return new Rotation2d(Math.atan2(speed.vyMetersPerSecond, speed.vxMetersPerSecond));
    }

    public Pose2d getRelativePose(){
        //return a pose that is offset by the difference between 
        //the original note location and the current one
        return new Pose2d(r.drive.getPose().getTranslation().plus(robotOffset),
                          r.drive.getPose().getRotation());
    }
}
