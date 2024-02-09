package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class CMDShoot {
    //simple shoot cmd
    //button on ctrl board and trigger on flysky when cam not enabled
    //start motors, set angle, move gate, wait, stop

    static double shootWaitTime;

    public static Command simpleShoot(RobotContainer r){
        Command c = new RunCommand( () -> r.shooter.fixedPrime());
                c = c.until(() -> r.shooter.checkAngleError() && r.shooter.checkRPMError());
                c = c.andThen(new InstantCommand(() -> r.gather.setGatePower(1)));
                c = c.andThen(new WaitCommand(shootWaitTime)); 
                c = c.finallyDo(() -> {r.shooter.goHome(); r.gather.setGatePower(0);});



                c.addRequirements(r.shooter, r.gather);
                c.setName("CmdSimpleShoot");
        return c;
    }
}
