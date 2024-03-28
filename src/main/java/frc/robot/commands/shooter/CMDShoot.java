package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.auton.Locations;

public class CMDShoot {
    //simple shoot cmd
    //button on ctrl board and trigger on flysky when cam not enabled
    //start motors, set angle, move gate, wait, stop

    static double shootWaitTime = 0.5;


    public static Command simpleShoot(RobotContainer r){
        Command c = new SequentialCommandGroup(
            new RunCommand(() -> {r.shooter.fixedPrime();
                                  r.state.isPrime = true;
                                }, r.shooter)
                .until(() -> r.shooter.checkAngleError() 
                          && r.shooter.checkRPMError() 
                          && r.inputs.shootTriggerSWH.getAsBoolean()),
            new InstantCommand(() -> r.gather.setGatePower(1), r.gather),
            new WaitCommand(shootWaitTime),
            new InstantCommand(() -> {r.shooter.goHome();
                                      r.gather.setGatePower(0); 
                                      r.state.hasNote = false;}, 
                                            r.shooter, r.gather),
            new WaitUntilCommand(() -> !r.inputs.shootTriggerSWH.getAsBoolean()),
            new InstantCommand(() -> r.state.isPrime = false)
            //make sure trigger is released so it doesnt immediately run again
        );//.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

        c.setName("CmdSimpleShoot");
        return c;
        
        /*
        new RunCommand( () -> r.shooter.fixedPrime());
                c = c.until(() -> r.shooter.checkAngleError() && r.shooter.checkRPMError() && r.inputs.shootTriggerSWH.getAsBoolean());
                c = c.andThen(new InstantCommand(() -> {r.gather.setGatePower(1); r.state.hasNote = false;}));
                c = c.andThen(new WaitCommand(shootWaitTime)); 
                c = c.andThen(() -> {r.shooter.goHome(); r.gather.setGatePower(0);});
                c = c.finallyDo(() -> {r.shooter.setShootPower(0); });

                c.addRequirements(r.shooter, r.gather);
                c.setName("CmdSimpleShoot");
        return c;
        */
    }

    public static Command simpleCtrlBoardShoot(RobotContainer r, Trigger t){
        Command c = new SequentialCommandGroup(
            new RunCommand(() -> {r.shooter.commandPrime(r.shooter.k.ctrlBoardShootAngle, r.shooter.k.ctrlBoardShootSpeed);
                                  r.state.isPrime = true;
                                }, r.shooter)
                .until(() -> r.shooter.checkAngleError() 
                          && r.shooter.checkRPMError() 
                          && t.getAsBoolean()),
            new InstantCommand(() -> r.gather.setGatePower(1), r.gather),
            new WaitCommand(shootWaitTime),
            new InstantCommand(() -> {r.shooter.goHome();
                                      r.gather.setGatePower(0); 
                                      r.state.hasNote = false;}, 
                                            r.shooter, r.gather),
            new WaitUntilCommand(t.negate()),
            new InstantCommand(() -> r.state.isPrime = false)
            //make sure trigger is released so it doesnt immediately run again
        );//.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

        c.setName("CmdSimpleShoot");
        return c;
        
    }

    public static Command simpleAmpShoot(RobotContainer r){
        Command c = new SequentialCommandGroup(
            new RunCommand(() -> {r.shooter.setAngle(102);
                                  r.shooter.setRPM(1200);
                                  r.gather.setGatePosition(0.2);},//increment the gate position to not pinch the note
                                         r.shooter, r.gather)
                .until(() -> r.shooter.checkAngleError() 
                          && r.shooter.checkRPMError() 
                          && r.inputs.shootTriggerSWH.getAsBoolean()),
            new InstantCommand(() -> r.gather.setGatePower(1), r.gather),
            new WaitCommand(shootWaitTime),
            new InstantCommand(() -> {r.shooter.goHome();
                                      r.gather.setGatePower(0); 
                                      r.state.hasNote = false;}, 
                                            r.shooter, r.gather),
            new WaitUntilCommand(() -> !r.inputs.shootTriggerSWH.getAsBoolean())
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        
        c.setName("CmdSimpleAmpShoot");
        return c;
    }


    //TODO: do we need this? ungather does this already
    /*
    public static Command unShoot(RobotContainer r){
        Command c = (new RunCommand(() -> {r.shooter.goHome(); r.gather.setGatePower(-1);}));
                c = c.finallyDo(() -> {r.gather.setGatePower(0);});

                c.addRequirements(r.shooter, r.gather);
                c.setName("CmdUnShoot");
        return c;
    }*/

    /*
    public static Command fixedPrime(RobotContainer r){
        Command c = new FunctionalCommand( () -> {}},
                                           () -> r.shooter.fixedPrime(),
                                           (interrupted) -> {},
                                           () -> !r.state.isPrime,
                                           r.shooter, r.gather);

        c = c.withInterruptBehavior(InterruptionBehavior.kCancelSelf);
        c.setName("FixedPrime");

        return c;
    }
    */

    public static Command visionShoot(RobotContainer r){
        Command c = new SequentialCommandGroup(
            new RunCommand(() -> {r.shooter.distancePrime(Locations.tagSpeaker.getDistance(r.drive.getPose().getTranslation()));
                                  r.state.isPrime = true;
                                }, r.shooter)
                .until(() -> r.shooter.checkAngleError() 
                          && r.shooter.checkRPMError() 
                          && r.drive.atAngleSetpoint
                          && r.inputs.shootTriggerSWH.getAsBoolean()),
            new InstantCommand(() -> r.gather.setGatePower(1), r.gather),
            new WaitCommand(shootWaitTime),
            new InstantCommand(() -> {r.shooter.goHome();
                                      r.gather.setGatePower(0); 
                                      r.state.hasNote = false;}, 
                                            r.shooter, r.gather),
            new WaitUntilCommand(() -> !r.inputs.shootTriggerSWH.getAsBoolean()),
            new InstantCommand(() -> r.state.isPrime = false)
            //make sure trigger is released so it doesnt immediately run again
        );
        
        c.setName("VisionShoot");
        return c;    
    }

    /*
    public static Command autonShoot(RobotContainer r){
        Command c = new SequentialCommandGroup(
            new RunCommand(() -> {r.shooter.setAngle(60);
                                  r.shooter.setRPM(4000);})
                    .raceWith(new WaitCommand(0.3)),
            new InstantCommand(() -> r.gather.setGatePower(1)),
            new WaitCommand(0.3),
            new InstantCommand(() -> {r.gather.setGatePower(0);
                                      r.shooter.goHome();})
        );

        c.addRequirements(r.shooter, r.gather);
        c.setName("CmdAutonShoot");
        return c;
    }
    */
}
