package frc.robot.commands.slappah;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.RobotContainer;
import frc.robot.commands.gather.CmdGather;

public class CmdTransfer {

    static double startupDelay = 0.2;

    //slappah positions in degrees
    static double slapHomePos = 0;
    static double slapTransferPos = 8;
    static double slapPreTransPos = 27;
    static double slapPreClimbPos = 20;//unused, just use preTrans
    static double slapPreAmpPos = 64;
    static double slapAmpScorePos = 72;
    static double slapPreTrapPos = 75;//unused, go straight to trap score
    static double slapTrapScorePos = 72; //TODO: find actual number

    //shooter positions in degrees
    static double shootPreTransPos = 60;
    static double shootTransPos = 98;

    //transfer
    static double shootPower = 0.12;
    static double gatePower = 0.8;
    static double transferPower = 0.3;
    static double shooterCurrentLim = 30;
    static double extraTransfer = 10;

    //unTransfer
    static double unShootPower = -0.12;
    static double unGatePower = -0.3;
    static double unTransferPower = -0.3;
    static double extraGate = -1;

    public static Command unTransferFull(RobotContainer r){
        Command c = new SequentialCommandGroup(setup(r), unTransfer(r), end(r));
                c = c.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
                
        c.addRequirements(r.shooter, r.slappah, r.gather);
        c.setName("CmdUnTransfer");

        return c;
    }

    public static Command unTransfer(RobotContainer r){
                Command transfer = new SequentialCommandGroup(
                        new InstantCommand(() -> {r.shooter.setShootPower(unShootPower);
                                                  r.gather.setGatePower(unGatePower); 
                                                  r.slappah.setTransferPower(unTransferPower);
                                                 }),
                        new PrintCommand("stage 4"),
                        new WaitCommand(startupDelay),
                        new WaitUntilCommand(() -> r.gather.getGateCurrent() > CmdGather.detectGateCurrent)
                                            .raceWith(new WaitCommand(5)),//make sure we cant get stuck here
                        new PrintCommand("stage 5"),
                        new InstantCommand(() -> {r.shooter.setShootPower(0);
                                                  r.gather.setGatePower(0);
                                                  r.slappah.setTransferPower(0);
                                                  r.state.hasTransfer = false;}),
                        new PrintCommand("stage 6")
                        );

        return transfer;
    }

    public static Command transfer(RobotContainer r){
        Command transfer = new SequentialCommandGroup(
                        new InstantCommand(() -> {r.shooter.setShootPower(shootPower);
                                                  r.gather.setGatePower(gatePower); 
                                                  r.slappah.setTransferPower(transferPower);
                                                 }),
                        new PrintCommand("stage 4"),
                        new WaitCommand(startupDelay),
                        new WaitUntilCommand(() -> r.shooter.getShooterCurrent() > shooterCurrentLim)
                                            .raceWith(new WaitCommand(1)),//dont get stuck
                        new WaitUntilCommand(() -> r.shooter.getShooterCurrent() < shooterCurrentLim)
                                            .raceWith(new WaitCommand(3)),//make sure we cant get stuck here
                        new PrintCommand("stage 5"),
                        new InstantCommand(() -> {r.shooter.setShootPower(0);
                                                  r.gather.setGatePower(0);
                                                  r.slappah.setTransferPower(0);
                                                  r.state.hasTransfer = true;}),
                        new PrintCommand("stage 6")
                        );

        return transfer;

    }

    public static Command transferForAmp(RobotContainer r){
        Command c = new SequentialCommandGroup(setup(r), transfer(r), end(r));
        c = c.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        c.addRequirements(r.shooter, r.slappah, r.gather);
        c.setName("CmdTransfer");

        return c;
    }

    public static Command setup(RobotContainer r){
        //move shooter and arm to the right angle

        Command setUp = new SequentialCommandGroup(
                    new InstantCommand(() -> {r.shooter.setAngle(shootPreTransPos);
                                              r.slappah.setAngle(slapPreTransPos);
                                            }),
                    new PrintCommand("stage 1"),
                    new WaitUntilCommand(r.slappah::checkAngleError),
                    new InstantCommand(() -> r.shooter.setAngle(shootTransPos)),
                    new PrintCommand("stage 2"),
                    new WaitUntilCommand(r.shooter::checkAngleError),
                    new InstantCommand(() -> r.slappah.setAngle(slapTransferPos)),
                    new PrintCommand("stage 3"),
                    new WaitUntilCommand(r.slappah::checkAngleError)
                    );

        return setUp;
    }

    private static Command end(RobotContainer r){
        Command end = new SequentialCommandGroup(
                      new InstantCommand(() -> r.slappah.setAngle(slapPreTransPos)),
                      new PrintCommand("stage 7"),
                      new WaitUntilCommand(r.slappah::checkAngleError),
                      new InstantCommand(r.shooter::goHome),
                      new PrintCommand("stage 8"),
                      new WaitUntilCommand(r.shooter::checkAngleError),
                      new InstantCommand(() -> r.slappah.setAngle(slapHomePos)),
                      new PrintCommand("stage done")
        );

        return end;
    }

    public static Command goToTrap(RobotContainer r){
        Command move = new InstantCommand(() -> r.slappah.setAngle(slapTrapScorePos));

        return move;
    }
}
