package frc.robot.commands.climber;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.CmdDrive;
import frc.robot.commands.slappah.CmdTransfer;

public class CmdClimb {

    static double climbPower = 0.75;
    static double unClimbPower = -.2;

    static double pushAgainstWallPower = 0.15;

    public static Command simpleWinch(RobotContainer r){
        Command c = new RunCommand( () -> r.climber.evenClimb(climbPower));
                c = c.finallyDo(() -> r.climber.setWinchPower(0, 0));

        c.addRequirements(r.climber);
        c.setName("CmdSimpleWinch");

        return c;
    }

    public static Command testClimb(RobotContainer r){
        Command c = new FunctionalCommand   (() -> {}, 
                                            () -> {r.climber.setTestPower();
                                                   r.slappah.setAnglePwr(0.07);}, 
                                            (interrupted) -> {r.climber.setWinchPower(0, 0);
                                                              r.slappah.setAnglePwr(0);}, 
                                            () -> false);

        c.addRequirements(r.climber, r.slappah);
        c.setName("CmdTestClimb");

        return c;
    }
    
    public static Command simpleClimb(RobotContainer r){
        Command climb = new SequentialCommandGroup(
            new SequentialCommandGroup(
                CmdTransfer.setup(r),//go to transfer pos
                //drive under chain manually(in the auto version this would be a command)
                waitForShootToggle(r),
                CmdTransfer.transfer(r),//transfer note
                waitForShootToggle(r)
            ).deadlineWith(new CmdDrive(r)),//allow joystick driving
            CmdTransfer.goToTrap(r),//raise the arm and drive back (really coast and let the arm push us)
            new WaitUntilCommand(r.slappah::checkAngleError)
                            .deadlineWith(new InstantCommand(() -> r.drive.swerveDrivePwr(new ChassisSpeeds(-0.01,0,0), false))),
            new InstantCommand(() -> r.slappah.setAnglePwr(pushAgainstWallPower)), //force the arm against the wall to maintain robot pitch while climbing
            waitForShootToggle(r),//wait for trigger before actually winching
            new RunCommand(() -> r.climber.triggerEvenClimb(climbPower))
            

            
        );

        return climb;
    }

    private static Command waitForShootToggle(RobotContainer r){
        Command wait = new SequentialCommandGroup(
            new WaitUntilCommand(() -> !r.inputs.shootTriggerSWH.getAsBoolean()),
            new WaitUntilCommand(() -> r.inputs.shootTriggerSWH.getAsBoolean())
        );

        return wait;
    }
}
