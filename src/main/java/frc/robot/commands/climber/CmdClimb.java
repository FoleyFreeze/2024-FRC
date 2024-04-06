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
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.slappah.CmdTransfer;
import frc.robot.subsystems.RoboState.ClimbState;

public class CmdClimb {

    static double winchShooterAngle = 92;

    static double climbPower = 0.3;
    static double unClimbPower = -.2;

    static double climbFinishedCurr;
    static double winchTurnsForHooksUp = 0.4375 + 0.4;
    static double winchTurnsToChain = 0.4375 - 0.4 + winchTurnsForHooksUp;
    static double winchTurnsToFinish = 2.15 + winchTurnsToChain;

    

    static double pushAgainstWallPower = 0.07;

    static double c2TurnsForHooksUp = -1.100; //was -.4365
    static double c2WinchTurnsToFinish = 1.65;
    static double c2ArmAnglePrep = 100; //get high enough to partially eject the note
    static double c2ArmAngleEnd = 110;//100
    static double c2ArmAngle = 65;     //was 80; on 3/28
    static double c2ShooterAngle = 103; //Was 95 on 3/20 bot tipping to far fwd

    public static Command deployClimb(RobotContainer r){
        return new SequentialCommandGroup(
            CmdTransfer.setup(r, false, true), //go to transfer pos but dont transfer yet
            new InstantCommand(() -> r.state.climbDeploy = ClimbState.DEPLOYED)
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public static Command undeployClimb(RobotContainer r){
        return new SequentialCommandGroup(
            CmdTransfer.end(r),
            new InstantCommand(() -> r.state.climbDeploy = ClimbState.NONE)
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public static Command undeployClimb2(RobotContainer r){
        Command c = new SequentialCommandGroup(
            new InstantCommand(() -> r.climber.setWinchPosition(0)),
            new WaitCommand(0.5),
            new InstantCommand(() -> r.shooter.goHome()),
            new WaitUntilCommand(() -> r.shooter.inputs.anglePosition < 60),
            new InstantCommand(() -> r.slappah.setAngle(0)),
            new InstantCommand(() -> r.state.climbDeploy = ClimbState.NONE)
        );

        c.setName("UndeployClimb2");
        return c;
    }

    public static Command deployClimb2(RobotContainer r){
        Command c = new SequentialCommandGroup(
            new InstantCommand(() -> r.slappah.setAngle(c2ArmAnglePrep),r.slappah),
            new WaitUntilCommand(() -> r.slappah.inputs.anglePosition > 80),
            new InstantCommand(() -> r.slappah.setTransferPosition(-5), r.slappah),
            new WaitCommand(0.25),//TODO: make longer?
            //new WaitUntilCommand(() -> r.slappah.inputs.anglePosition > 25),
            new InstantCommand(() -> r.shooter.setAngle(c2ShooterAngle), r.shooter),
            new InstantCommand(() -> r.climber.setWinchPosition(c2TurnsForHooksUp), r.climber),
            new WaitCommand(0.5),//wait for winches to clear arm
            new InstantCommand(() -> r.slappah.setAngle(c2ArmAngle), r.slappah),
            new InstantCommand(() -> r.state.climbDeploy = ClimbState.DEPLOYED)
        );
        c.setName("DeployClimb2");
        return c;
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

    public static Command hook(RobotContainer r){
        Command hook = new SequentialCommandGroup(
            CmdTransfer.transfer(r, true),
            CmdTransfer.goToPreTrap(r),
            new InstantCommand(() -> r.shooter.setAngle(winchShooterAngle)),
            new WaitUntilCommand(r.slappah::checkAngleError)
                            .deadlineWith(new RunCommand(() -> r.drive.swerveDrivePwr(new ChassisSpeeds(0.13,0,0), false), r.drive)),
            new InstantCommand(() -> r.drive.swerveDrivePwr(new ChassisSpeeds()), r.drive),
            new SequentialCommandGroup(
                new InstantCommand(() -> r.drive.swerveDrivePwr(new ChassisSpeeds(-0.12,0,0), false), r.drive),
                new WaitCommand(0.2),
                new InstantCommand(() -> r.drive.swerveDrivePwr(new ChassisSpeeds()), r.drive),
                new WaitCommand(0.2)
            ).deadlineWith(new SequentialCommandGroup(
                    new WaitCommand(0.1),
                    new InstantCommand(() -> r.climber.setWinchPosition(winchTurnsForHooksUp), r.climber)
                )),
            new WaitUntilCommand(() -> r.climber.checkWinchPosition()),
            new WaitUntilCommand(r.inputs.shootTriggerSWH.negate()),
            new InstantCommand(() -> r.state.climbDeploy=ClimbState.HOOKED)
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        
        hook.setName("Captain Hook");
        return hook;    
    }

    public static Command climb(RobotContainer r){
        Command climb = new SequentialCommandGroup(
            new InstantCommand(() -> r.slappah.setAnglePwr(pushAgainstWallPower), r.slappah), //force the arm against the wall to maintain robot pitch while climbing
            new WaitCommand(.5),
            new InstantCommand(() -> r.climber.setWinchPosition(winchTurnsToFinish)),
            new WaitUntilCommand(r.inputs.shootTriggerSWH.negate()),
            new InstantCommand(() -> r.state.climbDeploy = ClimbState.CLIMBED)
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

        climb.setName("Climb Cmd");
        return climb;
    }

    public static Command climb2(RobotContainer r){
        Command climb = new SequentialCommandGroup(
            new InstantCommand(() -> r.climber.setWinchPosition(c2WinchTurnsToFinish), r.climber),
            //wait for the robot to fall in to the wall
            new WaitUntilCommand(() -> r.drive.inputs.pitch.getRadians() < -0.5),
            new WaitCommand(0.5),
            //go all the way up so we fall all the way forward
            new InstantCommand(() -> r.slappah.setAngle(c2ArmAngleEnd), r.slappah),
            new InstantCommand(() -> r.state.climbDeploy = ClimbState.CLIMBED)
        );

        climb.setName("Climb2");
        return climb;
    }

    //note this is flipped now as we drive the note up to score
    public static Command shootTrap(RobotContainer r){
        /*Command shoot = new SequentialCommandGroup(
            new InstantCommand(() -> r.slappah.setTransferPower(-1), r.slappah), //score into the trap
            new WaitCommand(.5),
            new InstantCommand(() -> r.slappah.setTransferPower(0), r.slappah),
            new WaitUntilCommand(r.inputs.shootTriggerSWH.negate())
        );*/
        Command shoot = new RunCommand(() -> r.slappah.setTransferPower(1), r.slappah)
                    .finallyDo(() -> r.slappah.setTransferPower(0));

        shoot.setName("Trap Score");
        return shoot;
    }

    public static Command unshootTrap(RobotContainer r){
        Command unshoot = new RunCommand(() -> r.slappah.setTransferPower(-1), r.slappah)
                    .finallyDo(() -> r.slappah.setTransferPower(0));;

        unshoot.setName("Unshoot Trap");
        return unshoot;
    }

    /*
    public static Command simpleClimb(RobotContainer r){
        Command climb = new SequentialCommandGroup(
            new InstantCommand(() -> r.state.climbDeploy = true),
            new SequentialCommandGroup(
                CmdTransfer.setup(r),//go to transfer pos
                //drive under chain manually(in the auto version this would be a command)
                waitForShootToggle(r),
                CmdTransfer.transfer(r),//transfer note
                new InstantCommand(() -> {r.state.climbDeploy = true;
                                          r.state.hasTransfer = false;}),
                waitForShootToggle(r),
                CmdTransfer.goToPreTrap(r)//raise the arm and drive back (really coast and let the arm push us)
            ).deadlineWith(new CmdDrive(r)),//allow joystick driving
            new WaitUntilCommand(r.slappah::checkAngleError)
                            .deadlineWith(new RunCommand(() -> r.drive.swerveDrivePwr(new ChassisSpeeds(0.13,0,0), false), r.drive)),
            new InstantCommand(() -> r.drive.swerveDrivePwr(new ChassisSpeeds()), r.drive),
            new InstantCommand(() -> r.climber.setWinchPosition(winchTurnsToChain), r.climber),
            new WaitUntilCommand(() -> r.climber.checkWinchPosition()),
            new RunCommand(() -> r.drive.swerveDrivePwr(new ChassisSpeeds(-0.15,0,0), false), r.drive)
                .raceWith(new WaitCommand(0.4)),
            new InstantCommand(() -> r.drive.swerveDrivePwr(new ChassisSpeeds()), r.drive),
            new WaitCommand(0.2),
            new InstantCommand(() -> r.climber.setWinchPosition(winchTurnsToChain), r.climber),
            new WaitUntilCommand(() -> r.climber.checkWinchPosition()),
            new InstantCommand(() -> r.slappah.setAnglePwr(pushAgainstWallPower), r.slappah), //force the arm against the wall to maintain robot pitch while climbing
            //waitForShootToggle(r),//wait for trigger before actually winching
            new RunCommand(() -> r.climber.triggerEvenClimb(), r.climber)
                .raceWith(waitForShootToggle(r)),
            new InstantCommand(() -> r.slappah.setTransferPower(-1), r.slappah), //score into the trap
            new WaitCommand(.5),
            new InstantCommand(() -> r.slappah.setTransferPower(0), r.slappah)
            
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .ignoringDisable(true);

        climb.setName("SimpleClimb");
        return climb;
    }
    */
 

    private static Command waitForShootToggle(RobotContainer r){
        Command wait = new SequentialCommandGroup(
            new WaitUntilCommand(() -> !r.inputs.shootTriggerSWH.getAsBoolean()),
            new WaitUntilCommand(() -> r.inputs.shootTriggerSWH.getAsBoolean())
        );

        return wait;
    }


}
