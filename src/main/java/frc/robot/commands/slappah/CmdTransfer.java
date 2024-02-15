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
    static double extraTransfer = 2;

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
                                    r.gather.setGatePosition(extraGate);})
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

        Command c = new SequentialCommandGroup(setup(r), unTransfer, end(r));
        
        c.addRequirements(r.shooter, r.slappah, r.gather);
        c.setName("CmdUnTransfer");

        return c;
    }

    public static Command transfer(RobotContainer r){
        Command transfer = new InstantCommand(() -> {r.shooter.setShootPower(shootPower);
                                                     r.gather.setGatePower(gatePower); 
                                                     r.slappah.setTransferPower(transferPower);})
                .andThen(new WaitCommand(startupDelay))
                .andThen(new WaitUntilCommand(() -> r.slappah.getTransferCurrent() > transferCurrentLim))
                .finallyDo(() -> {r.shooter.setShootPower(shootPower); 
                                    r.gather.setGatePower(gatePower); 
                                    r.slappah.setTransferPosition(extraTransfer);})
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
