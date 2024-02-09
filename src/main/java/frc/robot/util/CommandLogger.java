package frc.robot.util;

import java.util.HashSet;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class CommandLogger {
    
    private static HashSet<String> runningCommands = new HashSet<>(10);
    private static StringBuilder sb = new StringBuilder(128);

    public static void addCommand(Command c){
        String name = c.getName();
        
        runningCommands.add(name);
        sb.append(name);
        sb.append(" ");

        Logger.recordOutput("RunningCommands", sb.toString());
    }

    public static void removeCommand(Command c){
        runningCommands.remove(c.getName());

        sb.delete(0, sb.length());
        for(String s : runningCommands){
            sb.append(s);
            sb.append(" ");
        }

        Logger.recordOutput("RunningCommands", sb.toString());
    }

    public static void logScheduler(){
        CommandScheduler.getInstance().onCommandInitialize(CommandLogger::addCommand);
        CommandScheduler.getInstance().onCommandInterrupt(CommandLogger::removeCommand);
        CommandScheduler.getInstance().onCommandFinish(CommandLogger::removeCommand);
    }

}
