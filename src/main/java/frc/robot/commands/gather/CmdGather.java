package frc.robot.commands.gather;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;

public class CmdGather {
    
    static double startupTime = 0.2;
    static double extraIntakeTime = 0.3;

    static double intakePower = 0.4;
    static double gatePower = 0.3;
    static double reverseIntakePower = -0.5;
    static double reverseGatePower = -0.3;

    static double extraGateRevs = 3.85;

    static double detectGateCurrent = 12;

    public static Command gather (RobotContainer r){
        //start spinning things and wait for start up current to decay
        Command c = new WaitCommand(startupTime)
            .deadlineWith(new RunCommand( () -> {r.gather.setGatherPower(intakePower, gatePower);
                                                 r.shooter.goHome();}));
        //detect when the piece has made it to the gate wheel
        c = c.andThen(new WaitUntilCommand(() -> r.gather.getGateCurrent() > detectGateCurrent))
            .finallyDo(() -> r.gather.setGatePower(0));
        //move the piece to it's final holding position
        c = c.andThen(new InstantCommand(() -> {r.gather.setGatePosition(extraGateRevs);
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
                                            r.shooter.setShootPower(-0.15);
                                            r.shooter.goHome();}))
                         .finallyDo(() -> {r.gather.setGatherPower(0, 0); 
                                            r.shooter.setShootPower(0);
                                            r.state.hasNote = false;});

        c.addRequirements(r.gather, r.shooter);
        c.setName("CmdUngather");
        return c;
    }
}
