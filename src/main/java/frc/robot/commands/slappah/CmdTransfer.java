package frc.robot.commands.slappah;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.commands.gather.CmdGather;

public class CmdTransfer {

    static double startupDelay = 0.15;

    //slappah positions in degrees
    static double slapHomePos = -3;//0
    static double slapTransferPos = 5;//10
    static double slapPreTransPos = 34;
    static double slapPreClimbPos = 25;//unused, just use preTrans
    static double slapPreAmpPos = 90; // was on 3/20/2024 55;//90;
    static double slapAmpScorePos = 108;
    static double slapPreTrapPos = 75;
    static double slapTrapScorePos = 108; //TODO: find actual number

    //shooter positions in degrees
    static double shootPreTransPos = 60;
    static double shootTransPos = 100;
    static double shootUntransPos = 97;
    static double shootClimbPos = 95;

    //transfer
    static double shootPower = 0.30;
    static double gatePower = 0.8;
    static double transferPower = 0.35;
    static double shooterCurrentLim = 30;
    static double extraTransferGate = 0.3;
    static double transferBackup = -1.5;
    static double transferRotations = 6.0; //was 7.0; on 3/20/2024
    static double climbTransferRotations = 6.0;
    static double transferCurrentLimit = 4;

    //unTransfer
    static double unShootPower = -0.75;//55
    static double unGatePower = -0.3;
    static double unTransferPower = -.6; //was on 3/20 -0.3;
    static double extraReverseGate = -1;

    //score
    static double scoreTransferPower = -1;
    static double scoreWaitTime = 0.6;
    static double scoreAnglePower = 0.1;

    public static Command unTransferFull(RobotContainer r, Trigger t){
        Command c = new SequentialCommandGroup(setup(r, true), 
                                               unTransfer(r), 
                                               end(r),
                                               new WaitUntilCommand(() -> !t.getAsBoolean()));
                c = c.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
                
        c.setName("CmdUnTransfer");

        return c;
    }

    public static Command unTransfer(RobotContainer r){
                Command transfer = new SequentialCommandGroup(
                        new InstantCommand(() -> r.shooter.setShootPower(unShootPower), r.shooter),
                        new WaitCommand(0.2),
                        new InstantCommand(() -> {r.shooter.setShootPower(unShootPower);
                                                  r.gather.setGatePower(unGatePower); 
                                                  r.slappah.setTransferPower(unTransferPower);
                                                 }, r.shooter, r.gather, r.slappah),
                        //new PrintCommand("stage 4"),
                        new WaitCommand(startupDelay),
                        new WaitUntilCommand(() -> r.gather.getGateCurrent() > CmdGather.detectGateCurrent)
                                            .raceWith(new WaitCommand(2)),//make sure we cant get stuck here
                        //new PrintCommand("stage 5"),
                        new InstantCommand(() -> {r.shooter.setShootPower(0);
                                                  r.gather.setGatePosition(extraReverseGate);
                                                  r.slappah.setTransferPower(0);
                                                  r.state.hasTransfer = false;},
                                                        r.shooter, r.gather, r.slappah)
                        //new PrintCommand("stage 6")
                        );

        return transfer;
    }

    public static Command transfer(RobotContainer r, boolean isClimb){
        double rotations;
        if(isClimb){
            rotations = climbTransferRotations;
        } else {
            rotations = transferRotations;
        }
        
        Command transfer = new SequentialCommandGroup(
                        new InstantCommand(() -> {r.shooter.setShootPower(shootPower);
                                                  r.gather.setGatePower(gatePower); 
                                                  r.slappah.setTransferPower(transferPower);
                                                 }, r.shooter, r.gather, r.slappah),
                        //new PrintCommand("stage 4"),
                        new WaitCommand(startupDelay),
                        //new WaitUntilCommand(() -> r.shooter.getShooterCurrent() > shooterCurrentLim)
                        //new WaitUntilCommand(() -> r.slappah.getTransferCurrent() > transferCurrentLimit)
                        new WaitUntilCommand(() -> !r.gather.inputs.proxSensor)
                                            .raceWith(new WaitCommand(1)),//dont get stuck
                        //new WaitUntilCommand(() -> r.shooter.getShooterCurrent() < shooterCurrentLim)
                        new InstantCommand(() -> r.slappah.setTransferPosition(rotations)),
                        new WaitUntilCommand(r.slappah::checkTransferError)
                                            .raceWith(new WaitCommand(0.75)),//make sure we cant get stuck here
                        //new PrintCommand("stage 5"),
                        new InstantCommand(() -> {r.shooter.setShootPower(0);
                                                  r.gather.setGatePower(0);
                                                  //r.slappah.setTransferPosition(transferBackup);
                                                }, r.shooter, r.gather, r.slappah)
                        //new PrintCommand("stage 6")
                        );

        return transfer;

    }

    public static Command transferForAmp(RobotContainer r, Trigger t){
        Command c = new SequentialCommandGroup(setup(r, false),
                                               transfer(r, false), 
                                               new InstantCommand(() -> r.state.hasTransfer = true),
                                               end(r),
                                               new WaitUntilCommand(() -> !t.getAsBoolean()));
        c = c.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        c.setName("CmdTransfer");

        return c;
    }

    public static Command setup(RobotContainer r, boolean isUntransfer){
        return setup(r, isUntransfer, false);
    }

    public static Command setup(RobotContainer r, boolean isUntransfer, boolean isClimb){
        //move shooter and arm to the right angle
        double shooterAngle;
        if(isClimb){
            shooterAngle = shootClimbPos;
        } else if(isUntransfer){
            shooterAngle = shootUntransPos;
        } else {
            shooterAngle = shootTransPos;
        }

        Command setUp = new SequentialCommandGroup(
                    new InstantCommand(() -> {r.shooter.setAngle(shootPreTransPos);
                                              r.slappah.setAngle(slapPreTransPos);
                                              r.gather.setGatePosition(extraTransferGate);
                                            }, r.shooter, r.slappah, r.gather),
                    //new PrintCommand("stage 1"),
                    new WaitUntilCommand(r.slappah::checkAngleError),
                    new InstantCommand(() -> r.shooter.setAngle(shooterAngle), r.shooter),
                    //new PrintCommand("stage 2"),
                    new WaitUntilCommand(r.shooter::checkAngleError),
                    new InstantCommand(() -> {r.slappah.setAngle(slapTransferPos);
                                              r.gather.setGatePosition(extraReverseGate);
                                             }, r.slappah, r.gather),
                    //new PrintCommand("stage 3"),
                    //TODO: figure out a better way for this
                    new WaitUntilCommand(r.slappah::checkAngleError)
                        .raceWith(new SequentialCommandGroup(
                            new WaitCommand(0.5),
                            new WaitUntilCommand(() -> r.slappah.inputs.anglePosition < slapPreTransPos),
                            new PrintCommand("%%%%%%%%%% The Arm Failed to make it to the transfer position %%%%%%%%%%")
                        ))
                    );

        return setUp;
    }

    public static Command end(RobotContainer r){
        Command end = new SequentialCommandGroup(
                      new InstantCommand(() -> r.slappah.setAngle(slapPreAmpPos), r.slappah), //slapPreTransPos
                      //new PrintCommand("stage 7"),
                      new WaitUntilCommand(() -> r.slappah.inputs.anglePosition > slapPreTransPos-4),
                      new InstantCommand(r.shooter::goHome, r.shooter),
                      //new PrintCommand("stage 8"),
                      //new WaitUntilCommand(r.shooter::checkAngleError),
                      new InstantCommand(() -> r.slappah.setAngle(slapPreAmpPos), r.slappah)
                      //new PrintCommand("stage done")
        );

        return end;
    }

    public static Command goToTrap(RobotContainer r){
        Command move = new InstantCommand(() -> r.slappah.setAngle(slapTrapScorePos), r.slappah);

        return move;
    }

    public static Command goToPreTrap(RobotContainer r){
        Command move = new InstantCommand(() -> r.slappah.setAngle(slapPreTrapPos), r.slappah);

        return move;
    }

    public static Command goToPreAmp(RobotContainer r){
        //if we are down, go up, otherwise go down
        Command c = new ConditionalCommand(
            new InstantCommand(() -> r.slappah.setAngle(slapPreAmpPos), r.slappah),
            new InstantCommand(() -> r.slappah.setAngle(slapHomePos), r.slappah),
            () -> r.slappah.inputs.anglePosition < (slapPreAmpPos + slapHomePos) / 2.0
        );

        return c;
    }

    public static Command scoreInAmp(RobotContainer r){
        Command c = new SequentialCommandGroup(
            new InstantCommand(() -> r.slappah.setAngle(slapAmpScorePos), r.slappah),
            new WaitUntilCommand(() -> r.slappah.checkAngleError()),
            new InstantCommand(() -> {r.slappah.setTransferPower(scoreTransferPower);
                                      r.slappah.setAnglePwr(scoreAnglePower);
                                     }, r.slappah),
            new WaitCommand(scoreWaitTime), 
            new WaitUntilCommand(r.inputs.shootTriggerSWH.negate()), //wait until shoot released
            new InstantCommand(() -> {r.slappah.setAngle(slapHomePos);
                                      r.slappah.setTransferPower(0);
                                      r.state.hasNote = false;
                                      r.state.hasTransfer = false;
                                     }, r.slappah)
        );

        return c;
    }

}
