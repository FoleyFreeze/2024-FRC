package frc.robot.auton;

import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.gather.CmdGather;

public class CmdAuton {

    static PathConstraints constraints;
    static Thread calcThread;
    
    public static void selectedAuto(RobotContainer r, int a, int b, int c, int d, int e, int f, int g, int h, int total,
                                        Consumer<Command> callback){
        int noteOrder[] = new int[8];
        sort(noteOrder, a, 1);
        sort(noteOrder, b, 2);
        sort(noteOrder, c, 3);
        sort(noteOrder, d, 4);
        sort(noteOrder, e, 5);
        sort(noteOrder, f, 6);
        sort(noteOrder, g, 7);
        sort(noteOrder, h, 8);

        //generate the command in a new thread
        calcThread = new Thread(() -> calcSelectedAutoPath(r, noteOrder, callback));
        calcThread.setName("AutonGenerationThread");
        calcThread.start();        
    }

    public static void cancelGeneration(){
        if(calcThread != null && calcThread.isAlive()){
            calcThread.interrupt();
        }
    }

    public static void calcSelectedAutoPath(RobotContainer r, int[] noteOrder, Consumer<Command> callback){
        SequentialCommandGroup cmd = new SequentialCommandGroup();

        //Step1: shoot the note you start with
        //cmd.addCommands(null);

        //Step2: for each note in order do:
        //     A: create a path to it
        //     B: add a command to follow that path
        //          i: calculate the angle to point the robot to by looking at the vector to the note
        //     C: add a command to run the gatherer
        //     D: Find the best shooting position for that note
        //          i: this will be the shooting position that is closest to the current note and the next one (if there is one)
        //     E: create a path to the shooting position
        //          i: calculate the angle to point the robot to by looking at the vector to the speaker
        //     F: add a command to follow that path
        //     G: add a command to prime the shooter
        //     H: Shoot

        Translation2d startPosition = r.drive.getPose().getTranslation();

        for(int i=0;i<noteOrder.length;i++){
            if(noteOrder[i] == 0){
                //stop if there is no note at this position
                break;
            }

            Translation2d dest = Locations.notes[noteOrder[i]-1];

            Pathfinding.setStartPosition(startPosition);
            Pathfinding.setGoalPosition(dest);

            //let the path generate
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                callback.accept(null);
                return;
            }

            //calc the angle we want to gather the note from
            Translation2d vecToNote = dest.minus(startPosition);
            
            GoalEndState endState = new GoalEndState(0, vecToNote.getAngle());
            if(Pathfinding.isNewPathAvailable()){
                PathPlannerPath path = Pathfinding.getCurrentPath(constraints, endState);
                //create a command to drive this path
                //cmd.addCommands();
            } else {
                callback.accept(null);
                return;
            }

            //save this for the next path
            startPosition = dest;


            Translation2d shootPosition = null;// = findBestShootPos(dest, Locations.notes[noteOrder[i+1]])

            Pathfinding.setStartPosition(startPosition);
            Pathfinding.setGoalPosition(shootPosition);

            //let the path generate
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                callback.accept(null);
                return;
            }

            //calc the angle we want to gather the note from
            Translation2d vecToSpeaker = shootPosition.minus(startPosition);
            
            endState = new GoalEndState(0, vecToSpeaker.getAngle());
            if(Pathfinding.isNewPathAvailable()){
                PathPlannerPath path = Pathfinding.getCurrentPath(constraints, endState);
                //create a command to drive this path
                //cmd.addCommands();
            } else {
                callback.accept(null);
                return;
            }

            Translation2d toSpeaker = Locations.tagSpeaker.minus(shootPosition);
            
        }

        //give the command back to the caller
        callback.accept(cmd);
    }


    private static void sort(int noteOrder[], int index, int note){
        if(index>0){
            noteOrder[index - 1] = note;
        }
    }


    public static void stopAll(RobotContainer r){
        r.shooter.goHome();
        r.gather.setGatherPower(0, 0);
    }
}
