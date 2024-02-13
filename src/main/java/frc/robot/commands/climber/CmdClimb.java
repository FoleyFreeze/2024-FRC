package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;

public class CmdClimb {

    static double climbPower = 0.75;

    public static Command simpleWinch(RobotContainer r){
        Command c = new RunCommand( () -> r.climber.balancedClimb(climbPower));

        c.addRequirements(r.climber);
        c.setName("CmdSimpleWinch");

        return c;
    }
}
