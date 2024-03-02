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

public class CmdTransfer {

    static double startupDelay = 0.25;

    //slappah positions in degrees
    static double slapHomePos = 0;
    static double slapTransferPos = 12;
    static double slapPreTransPos = 30;
    static double slapPreClimbPos = 20;
    static double slapPreAmpPos = 64;
    static double slapAmpScorePos = 72;
    static double slapPreTrapPos = 75;
    static double slapTrapScorePos = 95; 

    //shooter positions in degrees
    static double shootPreTransPos = 60;
    static double shootTransPos = 98;

    //transfer
    static double shootPower = 0.2;
    static double gatePower = 0.9;
    static double transferPower = 0.6;
    static double transferCurrentLim = 5;
    static double extraTransfer = 10;

    //unTransfer
    static double unShootPower = -.1;
    static double unGatePower = -.1;
    static double unTransferPower = -.1;
    static double gateCurrentLim = 8;
    static double extraGate = -2.5;

    public static Command unTransfer(RobotContainer r){
        Command unTransfer = new InstantCommand(() -> {r.shooter.setShootPower(unShootPower);
                                                       r.gather.setGatePower(unGatePower);
                                                       r.slappah.setTransferPower(unTransferPower);})
                .andThen(new WaitCommand(startupDelay))
                .andThen(new WaitUntilCommand(() -> r.gather.getGateCurrent() > gateCurrentLim))
                .finallyDo(() -> {r.shooter.setShootPower(unShootPower);
                                  r.gather.setGatePower(unGatePower);
                                  r.gather.setGatePosition(extraGate);
                                  r.state.hasTransfer = false;
                                })
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

        Command c = new SequentialCommandGroup(setup(r), unTransfer, end(r));
        
        c.addRequirements(r.shooter, r.slappah, r.gather);
        c.setName("CmdUnTransfer");

        return c;
    }

    public static Command transferForAmp(RobotContainer r){
        Command transfer = new SequentialCommandGroup(
                        new InstantCommand(() -> {r.shooter.setShootPower(shootPower);
                                                  r.gather.setGatePower(gatePower); 
                                                  r.slappah.setTransferPower(transferPower);
                                                 }),
                        new PrintCommand("stage 4"),
                        new WaitCommand(startupDelay),
                        new WaitUntilCommand(() -> r.slappah.getTransferCurrent() > transferCurrentLim)
                                            .raceWith(new WaitCommand(2)),//make sure we cant get stuck here
                        new InstantCommand(() -> r.slappah.setTransferPosition(extraTransfer)),
                        new PrintCommand("stage 5"),
                        new WaitUntilCommand(r.slappah::checkTransferError),
                        new InstantCommand(() -> {r.shooter.setShootPower(0);
                                                  r.gather.setGatePower(0);
                                                  r.state.hasTransfer = true;}),
                        new PrintCommand("stage 6")
                        );


        Command c = new SequentialCommandGroup(setup(r), transfer, end(r));
        c = c.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        c.addRequirements(r.shooter, r.slappah, r.gather);
        c.setName("CmdTransfer");

        return c;
    }

    private static Command setup(RobotContainer r){
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
                      new InstantCommand(() -> r.slappah.setAngle(slapPreAmpPos)),
                      new PrintCommand("stage done")
        );

        return end;
    }
}
