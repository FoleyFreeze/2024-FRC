package frc.robot.auton;

import java.util.Optional;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.gather.CmdGather;
import frc.robot.commands.shooter.CMDShoot;

public class ChoreoAuto {
    
    public static int autonShootCount = 0;

    public static Command getChoreoPathFromPathPlanner(String name, RobotContainer r){
        NamedCommands.registerCommand("AutonShoot", readyToShoot());
        autonShootCount = 0;
        return new PathPlannerAuto(name)
            .alongWith(new SequentialCommandGroup(
                shoot(r, -1, 60, 5000),
                CmdGather.gather(r),
                shoot(r, 0, 55, 5000),
                CmdGather.gather(r),
                shoot(r, 1, 55, 5000),
                CmdGather.gather(r),
                shoot(r, 2, 50, 5000)
            ))
        .finallyDo(() -> CmdAuton.stopAll(r));
                    
    }
    
    public static Command readyToShoot(){
        return new InstantCommand(() -> autonShootCount++);
    }

    public static Command shoot(RobotContainer r, int idx, double angle, double rpm){
        return new SequentialCommandGroup(
            new InstantCommand(() -> {r.shooter.setAngle(angle);
                                      r.shooter.setRPM(rpm);}),
            new ParallelCommandGroup(
                new WaitCommand(0.2),
                new WaitUntilCommand(() -> autonShootCount > idx)
            ),
            new InstantCommand(() -> r.gather.setGatePower(1)),
            new WaitCommand(0.2)
        );
    }

    
    public static Command getChoreoPath(String fileName, RobotContainer r){
        ChoreoTrajectory traj = Choreo.getTrajectory(fileName);


        return Choreo.choreoSwerveCommand(
            traj, 
            r.drive::getPose,
            new PIDController(5.0, 0.0, 0.0),   //X Controller (units are meters)
            new PIDController(5.0, 0.0, 0.0),   //Y Controller
            new PIDController(0.0, 0.0, 0.0),   //Rotate Controller (units are radians)
            (ChassisSpeeds speeds) ->
                r.drive.swerveDrivePwr(speeds, false), //robot relative speeds
            () -> {   
                Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == Alliance.Red;
            },
            r.drive
        );
    }

    public static Command getPathPlannerAuto(String filename, RobotContainer r){
        PathPlannerPath path = PathPlannerPath.fromPathFile(filename);
        return AutoBuilder.followPath(path);
    }

    

}


