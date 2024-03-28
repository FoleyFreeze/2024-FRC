package frc.robot.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.gather.CmdGather;

public class ChoreoAuto {
    
    public static int autonShootCount = 0;

    public static Command getPregen3NoteMid(String name, RobotContainer r){
        NamedCommands.registerCommand("AutonShoot", readyToShoot());
        return new SequentialCommandGroup(
            new WaitCommand(0.4),
            readyToShoot(),
            new PathPlannerAuto(name)
        )
        .alongWith(new SequentialCommandGroup(
            new InstantCommand(() -> autonShootCount = 0),
            shoot(r, 0, 55, 4500),
            CmdGather.autonGather(r),
            shoot(r, 1, 38, 4500),
            CmdGather.autonGather(r),
            shoot(r, 2, 40, 4500),
            CmdGather.autonGather(r),
            shoot(r, 3, 38, 4500)
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
            new WaitUntilCommand(() -> autonShootCount > idx),
            new InstantCommand(() -> r.gather.setGatePower(1)),
            new WaitCommand(0.2)
        );
    }

    
    public static Command getChoreoPath(String fileName, RobotContainer r){
        PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(fileName);

        return new SequentialCommandGroup(
            new InstantCommand(() -> r.drive.resetFieldOdometry(path.getPreviewStartingHolonomicPose())),
            AutoBuilder.followPath(path)
        );
    }

    public static Command getPathPlannerAuto(String filename, RobotContainer r){
        PathPlannerPath path = PathPlannerPath.fromPathFile(filename);
        return new SequentialCommandGroup(
            new InstantCommand(() -> r.drive.resetFieldOdometry(path.getPreviewStartingHolonomicPose())),
            AutoBuilder.followPath(path)
        );
    }

    

}


