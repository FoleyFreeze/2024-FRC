package frc.robot.commands.gather;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;

public class CmdGather {
    
    static double startupTime = 0.2;
    static double extraIntakeTime = 0.0;

    static double backwardsGatherPower = -.13;
    static double intakePower = 0.5;//0.4
    static double gatePower = 0.25;//0.15
    static double reverseIntakePower = -0.6;//0.5
    static double reverseGatePower = -0.5;//0.3


    static double extraGateRevsCurrent = 2.6;
    static double extraGateRevsSensor = 1.2;

    public static double detectGateCurrent = 15;

    public static Command gatherCurrent (RobotContainer r){
        //start spinning things and wait for start up current to decay
        Command c = new WaitCommand(startupTime)
            .deadlineWith(new RunCommand( () -> {r.gather.setGatherPower(intakePower, gatePower);
                                                 r.shooter.goHome();}));
        //detect when the piece has made it to the gate wheel
        c = c.andThen(new WaitUntilCommand(() -> r.gather.getGateCurrent() > detectGateCurrent))
            .finallyDo(() -> r.gather.setGatePower(0));
        //move the piece to it's final holding position
        c = c.andThen(new InstantCommand(() -> {r.gather.setGatePosition(extraGateRevsCurrent);
                                                r.state.hasNote = true;}));
        //intake pushes note all the way to the gate then stops; gate holds it at constant position
        c = c.andThen(new WaitCommand(extraIntakeTime))
            .finallyDo(() -> r.gather.setIntakePower(0));

        c.addRequirements(r.gather, r.shooter);
        c.setName("CmdGather");
        return c;
    }

    public static Command gather (RobotContainer r){
        /*
        //start spinning things and wait for start up current to decay
        Command c = new WaitCommand(startupTime)
            .deadlineWith(new RunCommand( () -> {r.gather.setGatherPower(intakePower, gatePower);
                                                 r.shooter.goHome();}, r.gather, r.shooter));
        //detect when the piece has made it to the gate wheel
        c = c.andThen(new WaitUntilCommand(() -> r.gather.inputs.proxSensor))
            .finallyDo(() -> r.gather.setGatePower(0));
        //move the piece to it's final holding position
        c = c.andThen(new InstantCommand(() -> {r.gather.setGatePosition(extraGateRevsSensor);
                                                r.state.hasNote = true;}), r.gather);
        //intake pushes note all the way to the gate then stops; gate holds it at constant position
        c = c.andThen(new WaitCommand(extraIntakeTime))
            .finallyDo(() -> r.gather.setIntakePower(0));
        */

        Command c = new SequentialCommandGroup(
            //start spinning things and wait for start up current to decay
            new WaitCommand(startupTime)
                .deadlineWith(new RunCommand( () -> {r.gather.setGatherPower(intakePower, gatePower);
                                                     r.shooter.goHome();}, r.gather, r.shooter)),
            //detect when the piece has made it to the gate wheel
            new WaitUntilCommand(() -> r.gather.inputs.proxSensor)
                .finallyDo(() -> r.gather.setGatePower(0)),
            //move the piece to it's final holding position
            new InstantCommand(() -> {r.gather.setGatePosition(extraGateRevsSensor);
                                      r.state.hasNote = true;}, r.gather),
            //intake pushes note all the way to the gate then stops; gate holds it at constant position
            new WaitCommand(extraIntakeTime)
                .finallyDo(() -> r.gather.setIntakePower(0))
        );
    
        c.setName("CmdGather");
        return c;
    }

    public static Command autonGather (RobotContainer r){
        //start spinning things and wait for start up current to decay
        Command c = new WaitCommand(startupTime)
            .deadlineWith(new RunCommand( () -> {r.gather.setGatherPower(intakePower, gatePower);
                                                 r.shooter.setAngle(r.shooter.k.homePosition);}));
        //detect when the piece has made it to the gate wheel
        c = c.andThen(new WaitUntilCommand(() -> r.gather.inputs.proxSensor))
            .finallyDo(() -> r.gather.setGatePower(0));
        //move the piece to it's final holding position
        c = c.andThen(new InstantCommand(() -> {r.gather.setGatePosition(extraGateRevsSensor);
                                                r.state.hasNote = true;}));
        //intake pushes note all the way to the gate then stops; gate holds it at constant position
        c = c.andThen(new WaitCommand(extraIntakeTime))
            .finallyDo(() -> r.gather.setIntakePower(0));

        c.addRequirements(r.gather, r.shooter);
        c.setName("CmdGather");
        return c;
    }

    public static Command unGather(RobotContainer r){
        Command c = (new RunCommand(() -> {r.gather.setGatherPower(reverseGatePower, reverseIntakePower);
                                            r.slappah.setTransferPower(1);
                                            r.shooter.setShootPower(-0.15);
                                            r.shooter.setAngle(r.shooter.k.homePosition);}))
                         .finallyDo(() -> {r.gather.setGatherPower(0, 0); 
                                            r.shooter.setShootPower(0);
                                            r.slappah.setTransferPower(0);
                                            r.state.hasNote = false;
                                            r.state.hasTransfer = false;});

        c.addRequirements(r.gather, r.shooter, r.slappah);
        c.setName("CmdUngather");
        return c;
    }

    public static Command backwardsGather(RobotContainer r){
        return new RunCommand(() -> r.gather.setIntakePower(backwardsGatherPower), r.gather);
    }
}
