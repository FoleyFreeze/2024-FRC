package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class CmdDriveNoteTraj extends Command{

    RobotContainer r;

    double maxVelocity = 3;//m/s
    double maxAccel = 3;//m/s/s
    double maxAngularAccel = 2;//rad/s
    double maxAngularVelocity = 2;//rad/s/s

    PathConstraints pathConstraints = new PathConstraints(maxVelocity, maxAccel, maxAngularVelocity, maxAngularAccel);

    //only recalc path if the note has moved more than x distance
    double recalcError = Units.inchesToMeters(8);
    //only recalc path if the note is further away than x distance
    double recalcBounds = Units.inchesToMeters(24);

    Command driveCommand;

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
        } else {
            driveCommand = null;
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
                Translation2d distToGoal = pathEndLocation.minus(r.drive.robotPose.getTranslation());

                Translation2d noteError = pathEndLocation.minus(r.vision.getCachedNoteLocation());
                Logger.recordOutput("Vision/NoteError", noteError);

                //if path has diverged too far, and we are not close, make a new one
                if(distToGoal.getNorm() > recalcBounds && noteError.getNorm() > recalcError) {
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
        pathEndLocation = r.vision.getCachedNoteLocation();
        Translation2d pathStart = r.drive.getPose().getTranslation();
        Translation2d pathVector = pathEndLocation.minus(pathStart);

        //modify endLocation to be 6in further
        Translation2d additionalDist = new Translation2d(Units.inchesToMeters(6), 0);
        additionalDist.rotateBy(r.drive.getAngle());
        pathEndLocation.plus(additionalDist);
        
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            new Pose2d(pathStart, getVelocityAngle(r.drive.getRelVelocity(), pathVector)),
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
            r.drive::getPose, 
            r.drive::getRelVelocity, 
            r.drive::swerveDriveVel, 
            r.drive.k.notePathFollowerConfig, 
            () -> false, 
            r.drive
        );
    }

    Rotation2d positiveY = new Rotation2d(Math.PI/2.0);
    Rotation2d negativeY = new Rotation2d(-Math.PI/2.0);

    private Rotation2d getVelocityAngle(ChassisSpeeds speed, Translation2d pathVector){
        if(Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond) < 0.2){
            //if we are not moving, start our path perpendicular to the target orientation
            System.out.println("Zero init vel");
            Rotation2d botAngle = r.drive.getAngle();
            return (pathVector.getAngle().minus(botAngle).getSin() > 0 ? positiveY : negativeY).plus(botAngle);
        } else {
            System.out.println("Nonzero init vel");
            ChassisSpeeds fieldRel = ChassisSpeeds.fromRobotRelativeSpeeds(speed, r.drive.getAngle());
            return new Rotation2d(Math.atan2(fieldRel.vyMetersPerSecond, fieldRel.vxMetersPerSecond));
        }
    }

    /*
    public Pose2d getRelativePose(){
        //return a pose that is offset by the difference between 
        //the original note location and the current one
        return new Pose2d(r.drive.getPose().getTranslation().plus(robotOffset),
                          r.drive.getPose().getRotation());
    }
    */
}
