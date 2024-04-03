package frc.robot.commands.gather;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.CmdDrive;
import frc.robot.commands.drive.CmdDriveNoteTraj;

public class CmdGather {
    
    static double startupTime = 0.2;
    static double extraIntakeTime = 0.0;

    static double backwardsGatherPower = -.13;
    static double intakePower = 0.5; //was 0.75;//0.4
    static double gatePower = 0.3;//0.25
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
        //timer for tracking time at high current
        Timer currTimer = new Timer();
        currTimer.start();

        Command c = new SequentialCommandGroup(
            //start spinning things and wait for start up current to decay
            new WaitCommand(startupTime)
                .deadlineWith(new RunCommand( () -> {r.gather.setGatherPower(intakePower, gatePower);
                                                     r.shooter.goHome();
                                                     r.slappah.setAngle(0);
                                                    }, r.gather, r.shooter, r.slappah)),
            //detect when the piece has made it to the gate wheel
            new ParallelDeadlineGroup(
                new WaitUntilCommand(() -> r.gather.inputs.proxSensor),
                //while we have not sensed a note, monitor intake current
                //and briefly back up the gatherer if stalled for too long
                new SequentialCommandGroup(
                    new InstantCommand(() -> currTimer.reset()),
                    new WaitUntilCommand(() -> {
                                                if(r.gather.inputs.intakeCurrentAmps > 20){
                                                    return currTimer.hasElapsed(0.5);
                                                } else {
                                                    currTimer.reset();
                                                    return false;
                                                }
                                               }),
                    new InstantCommand(() -> r.gather.setGatherPower(reverseIntakePower, reverseGatePower)),
                    new WaitCommand(0.15),
                    new InstantCommand(() -> r.gather.setGatherPower(intakePower, gatePower)),
                    new InstantCommand(() -> currTimer.reset())
                )
            ).finallyDo(() -> r.gather.setGatePower(0)),
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

    public static Command cameraGather(RobotContainer r){
        Command c = new ParallelDeadlineGroup(
            new ParallelRaceGroup(
                CmdGather.gather(r),
                //continue running gather for an extra second
                //to make sure that any in-process notes get fully gathered
                new SequentialCommandGroup(
                    new WaitUntilCommand(r.inputs.gatherTriggerSWE.negate()),
                    new WaitCommand(1)
                )
            ),
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    //run camera drive until trigger is released
                    //or the intake wheels see a spike
                    new CmdDriveNoteTraj(r),
                    new WaitUntilCommand(r.inputs.gatherTriggerSWE.negate()),
                    new SequentialCommandGroup(
                        new WaitCommand(startupTime),
                        new WaitUntilCommand(() -> r.gather.getCurrent() > 12),
                        new WaitCommand(.100) //wait 1/10s to intake note after gath spike
                    )
                ),
                //then immediately return control to the driver
                new CmdDrive(r)
            )
        );

        c.setName("CameraGather");
        return c;
    }

    public static Command autonGather (RobotContainer r){
        Command c = new SequentialCommandGroup(
            //start spinning things and wait for start up current to decay
            new WaitCommand(startupTime)
                .deadlineWith(new RunCommand( () -> {r.gather.setGatherPower(intakePower, gatePower);
                                                     r.shooter.setAngle(r.shooter.k.homePosition);}, r.gather, r.shooter)),
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

        c.setName("Auto Gather");
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
        Command c = new RunCommand(() -> r.gather.setIntakePower(backwardsGatherPower), r.gather);
        c.setName("BackUngather");
        return c;
    }
}
