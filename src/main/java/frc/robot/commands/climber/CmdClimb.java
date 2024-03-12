package frc.robot.commands.climber;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.CmdDrive;
import frc.robot.commands.slappah.CmdTransfer;

public class CmdClimb {

    static double climbPower = 0.75;
    static double unClimbPower = -.2;

    static double climbFinishedCurr;

    static double pushAgainstWallPower = 0.07;

    public static Command deployClimb(RobotContainer r){
        return new SequentialCommandGroup(
            CmdTransfer.setup(r), //go to transfer pos but dont transfer yet
            new InstantCommand(() -> r.state.climbDeploy = true)
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public static Command undeployClimb(RobotContainer r){
        return new SequentialCommandGroup(
            CmdTransfer.end(r),
            new InstantCommand(() -> r.state.climbDeploy = false)
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public static Command visionClimb(RobotContainer r){
        return new InstantCommand();
    }

    public static Command manualClimb(RobotContainer r){
        return new InstantCommand();
    }

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
                waitForShootToggle(r),
                CmdTransfer.goToPreTrap(r)//raise the arm and drive back (really coast and let the arm push us)
            ).deadlineWith(new CmdDrive(r)),//allow joystick driving
            new WaitUntilCommand(r.slappah::checkAngleError)
                            .deadlineWith(new InstantCommand(() -> r.drive.swerveDrivePwr(new ChassisSpeeds(-0.01,0,0), false), r.drive)),
            new InstantCommand(() -> r.slappah.setAnglePwr(pushAgainstWallPower), r.slappah), //force the arm against the wall to maintain robot pitch while climbing
            waitForShootToggle(r),//wait for trigger before actually winching
            new RunCommand(() -> r.climber.triggerEvenClimb(), r.climber)
                .raceWith(waitForShootToggle(r)),
            new InstantCommand(() -> r.slappah.setTransferPower(-1), r.slappah), //score into the trap
            new WaitCommand(.5),
            new InstantCommand(() -> r.slappah.setTransferPower(0), r.slappah)
            
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

        climb.setName("SimpleClimb");
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
