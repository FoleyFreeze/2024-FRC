package frc.robot.commands.slappah;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.RobotContainer;

public class CmdTransfer {

    static double startupDelay = .1;

    //slappah positions in degrees
    public double slapTransferPos = 5;
    public double slapPreTransPos = 25;
    public double slapPreClimbPos = 20;
    public double slapPreAmpPos = 75;
    public double slapAmpScorePos = 95;
    public double slapPreTrapPos = 75;
    public double slapTrapScorePos = 95; 

    //shooter positions in degrees
    public double shootPreTransPos = 60;
    public double shootTransPos = 90;

    //startUp
    static double transferPosArmAngle = 0;
    static double transferPosShootAngle = 0;

    //end
    static double shootHomeAngle = 0;
    static double armHomeAngle = 0;
    static double noteArmPos = 0;

    //transfer
    static double shootPower = .1;
    static double gatePower = .1;
    static double transferPower = .1;
    static double transferCurrentLim = 8;
    static double extraTransfer = 8;

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
                        new WaitCommand(startupDelay),
                        new WaitUntilCommand(() -> r.slappah.getTransferCurrent() > transferCurrentLim)
                        );

                transfer.finallyDo(() -> {r.shooter.setShootPower(shootPower); 
                                          r.gather.setGatePower(gatePower); 
                                          r.slappah.setTransferPosition(extraTransfer);
                                          r.state.hasTransfer = true;
                                         })
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

        Command c = new SequentialCommandGroup(setup(r), transfer, end(r));
        
        c.addRequirements(r.shooter, r.slappah, r.gather);
        c.setName("CmdTransfer");

        return c;
    }

    private static Command setup(RobotContainer r){
        //move shooter and arm to the right angle
        Command setUp = new FunctionalCommand(
            () -> {r.shooter.setAngle(transferPosShootAngle); r.slappah.setAngle(transferPosArmAngle);},//init
            () -> {},//execute
            (i) -> {if(i){r.shooter.setAngle(shootHomeAngle); r.slappah.setAngle(armHomeAngle);}},//end
            () -> r.slappah.checkAngleError() && r.shooter.checkAngleError());//isFinished

        return setUp;
    }

    private static Command end(RobotContainer r){
        Command end = new FunctionalCommand(
            () -> {r.shooter.setAngle(shootHomeAngle); r.slappah.setAngle(noteArmPos);},
            () -> {},
            (i) -> {},
            () -> r.slappah.checkAngleError() && r.shooter.checkAngleError());

        return end;
    }
}
