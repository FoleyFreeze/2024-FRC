package frc.robot.commands.slappah;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.RobotContainer;

public class CmdTransfer {

    public static Command transfer(RobotContainer r){
        Command c = new InstantCommand(() -> r.shooter.setAngle(0));
        c = c.alongWith(new InstantCommand(() -> r.slappah.setAngle(0)));
        c = c.andThen(new WaitUntilCommand(() -> r.slappah.checkAngleError() && r.shooter.checkAngleError()));

        c = c.andThen(new InstantCommand(() -> r.shooter.setRPM(0)));
        c = c.andThen(new WaitUntilCommand(() -> r.slappah.getTransferCurrent() > 0));
        c.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

        c = c.andThen(new InstantCommand(() -> r.shooter.setRPM(0)));
        c = c.andThen(new InstantCommand(() -> r.shooter.setAngle(0)));
        c = c.alongWith(new InstantCommand(() -> r.slappah.setAngle(0)));    




        return c;
    }
}
