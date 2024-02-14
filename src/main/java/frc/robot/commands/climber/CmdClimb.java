package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;

public class CmdClimb {

    static double climbPower = 0.75;
    static double unClimbPower = -.2;

    public static Command simpleWinch(RobotContainer r){
        Command c = new RunCommand( () -> r.climber.evenClimb(climbPower));
         c = c.finallyDo(() -> r.climber.setWinchPower(0, 0));

        c.addRequirements(r.climber);
        c.setName("CmdSimpleWinch");

        return c;
    }
    
    /* Commented out due to ratchets on the winchs
    public static Command unSimpleWinch(RobotContainer r){
        Command c = new RunCommand( () -> r.climber.evenClimb(unClimbPower));
        c = c.finallyDo(() -> r.climber.setWinchPower(0, 0));

        c.addRequirements(r.climber);
        c.setName("CmdUnSimpleWinch");

        return c;
    }*/
}
