package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.RobotContainer;

public class CMDShoot {
    //simple shoot cmd
    //button on ctrl board and trigger on flysky when cam not enabled
    //start motors, set angle, move gate, wait, stop

    static double shootWaitTime = 0.5;


    public static Command simpleShoot(RobotContainer r){
        Command c = new RunCommand( () -> r.shooter.fixedPrime());
                c = c.until(() -> r.shooter.checkAngleError() && r.shooter.checkRPMError() && r.inputs.shootTriggerSWH.getAsBoolean());
                c = c.andThen(new InstantCommand(() -> {r.gather.setGatePower(1); r.state.hasNote = false; r.state.isPrime = false;}));
                c = c.andThen(new WaitCommand(shootWaitTime)); 
                c = c.andThen(() -> {r.shooter.goHome();});
                c = c.finallyDo(() -> {r.shooter.setShootPower(0); r.gather.setGatePower(0);});

                c.addRequirements(r.shooter, r.gather);
                c.setName("CmdSimpleShoot");
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

    public static Command fixedPrime(RobotContainer r){
        Command c = new FunctionalCommand( () -> r.state.isPrime = true,
                                           () -> r.shooter.fixedPrime(),
                                           (interrupted) -> {},
                                           () -> !r.state.isPrime);

                c.addRequirements(r.shooter, r.gather);
                c.setName("fixedPrime");

        return c;
    }

    public static Command visionShoot(RobotContainer r){
        Command c = new RunCommand( () -> r.shooter.fixedPrime());
                c = c.until(() -> r.shooter.checkAngleError() && r.shooter.checkRPMError() && r.inputs.shootTriggerSWH.getAsBoolean());
                c = c.andThen(new InstantCommand(() -> {r.gather.setGatePower(1); r.state.hasNote = false;}));
                c = c.andThen(new WaitCommand(shootWaitTime)); 
                c = c.finallyDo(() -> {r.shooter.goHome(); r.gather.setGatePower(0); r.state.isPrime = false;});

                c.addRequirements(r.shooter, r.gather);
                c.setName("VisionPrime");

        return c;    
    }

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
}
