package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.List;

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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class CmdDriveNoteTraj extends Command{

    RobotContainer r;

    double maxVelocity;
    double maxAccel;
    double maxAngularAccel;
    double maxAngularVelocity;

    PathConstraints pathConstraints = new PathConstraints(maxVelocity, maxAccel, maxAngularVelocity, maxAngularAccel);

    Command driveCommand;

    public CmdDriveNoteTraj(RobotContainer r){
        this.r = r;
    }

    @Override
    public void initialize(){
        driveCommand = createPathFollower();
        driveCommand.initialize();
    }

    @Override
    public void execute(){
        //TODO: recreate path if note moved
        driveCommand.execute();
    }

    @Override 
    public void end(boolean interrupted){
        driveCommand.end(interrupted);
    }

    @Override
    public boolean isFinished(){
        return driveCommand.isFinished();
    }

    public Command createPathFollower(){
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            new Pose2d(r.drive.getPose().getTranslation(), getVelocityAngle(r.drive.getVelocity())),
            new Pose2d(r.vision.getNoteLocation(), r.drive.getAngle())
        );

        PathPlannerPath path = new PathPlannerPath(
            bezierPoints, 
            pathConstraints, 
            new GoalEndState(0, r.drive.getAngle())
        );

        return new FollowPathHolonomic(
            path, 
            r.drive::getPose, 
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
}
