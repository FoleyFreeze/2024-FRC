package frc.robot.auton;

import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.CmdDriveNoteTraj;
import frc.robot.commands.gather.CmdGather;

public class CmdAuton {

    static PathConstraints constraints = new PathConstraints(
        2, //vel m/s
        2, //accel m/s/s
        4, //vel rad/s
        4  //accel rad/s/s
    );

    static double driveToNoteThresh = Units.inchesToMeters(24);

    public static Command selectedAuto(RobotContainer r, 
                                       int a, int b, int c, int d, int e, int f, int g, int h, int total){
        int noteOrder[] = new int[8];
        sort(noteOrder, a, 1);
        sort(noteOrder, b, 2);
        sort(noteOrder, c, 3);
        sort(noteOrder, d, 4);
        sort(noteOrder, e, 5);
        sort(noteOrder, f, 6);
        sort(noteOrder, g, 7);
        sort(noteOrder, h, 8);

        //Step1: shoot the note you start with
        //Step2: for each note in order do:
        //     A: create a path to it
        //     B: add a command to follow that path
        //          i: calculate the angle to point the robot to by looking at the vector to the note
        //         ii: run the gatherer in parallel
        //        iii: if within x dist of the note and we see it then we should stop pathfinding and use the driveToNote function
        //     C: If we gathered a note, then:
        //          i: find the best shooting position for that note
        //              -> this will be the shooting position that is closest to the current note and the next one (if there is one)
        //         ii: create a path to the shooting position
        //        iii: calculate the angle to point the robot to by looking at the vector to the speaker
        //         iv: add a command to follow that path
        //          v: add a command to prime the shooter
        //         vi: Shoot
        //     C-else: If we did not gather a note, then someone stole it first, continue on to the next note
        //     
        //Step3: fin
        
        SequentialCommandGroup fullSequence = new SequentialCommandGroup();

        //shoot first note
        Pose2d startPose = r.drive.getPose();
        double shootDist = Locations.tagSpeaker.minus(startPose.getTranslation()).getNorm();

        fullSequence.addCommands(new SequentialCommandGroup(
            prime(r, shootDist),
            new WaitCommand(0.4),
            shoot(r)
        ));

        //for each note
        for(int i=0;i<noteOrder.length;i++){
            if(noteOrder[i] == 0){
                //stop if there is no note at this position
                break;
            }

            // ----------- Pathfind to Note -------------
            Translation2d noteLocation = Locations.notes[noteOrder[i] - 1];
            Translation2d vecToNote = noteLocation.minus(startPose.getTranslation());
            Pose2d targetPose = new Pose2d(noteLocation, vecToNote.getAngle());

            Command pathFindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, 0.0
            );

            //construct the complex pathfind to note and gather it with vision command
            Command noteCommand = pathFindingCommand
                //until we are close to the note (and vision sees a note that close)
                .until(() -> {var botLoc = r.drive.getPose().getTranslation();
                        return botLoc.minus(noteLocation).getNorm() < driveToNoteThresh
                            && r.vision.hasNoteImage() 
                            && botLoc.minus(r.vision.getCachedNoteLocation()).getNorm() < driveToNoteThresh;}) 
                .andThen(new CmdDriveNoteTraj(r)
                    .raceWith(new WaitCommand(2))) //give it 4 seconds before moving on to the next note
            .raceWith(CmdGather.gather(r));

            fullSequence.addCommands(noteCommand);

            
            // ----------- Pathfind to Shoot -------------

            //determine best shooting location
            Translation2d nextNoteLoc = null;
            if(i+1 < noteOrder.length){
                if(noteOrder[i+1] > 0){
                    nextNoteLoc = Locations.notes[noteOrder[i+1] - 1];
                }
            }
            Translation2d shootLoc = getBestShootLocation(Locations.shootingPositions, noteLocation, nextNoteLoc);
            Translation2d vecToSpeaker = Locations.tagSpeaker.minus(shootLoc);
            targetPose = new Pose2d(shootLoc, vecToSpeaker.getAngle());

            pathFindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, 0.0
            );

            Command shootCommand = new SequentialCommandGroup(
                prime(r, vecToSpeaker.getNorm()),
                pathFindingCommand,
                shoot(r)
            );

            //only run the shoot sequence if we successfully gathered a note
            fullSequence.addCommands(shootCommand.onlyIf(() -> r.state.hasNote));
        }

        return fullSequence.finallyDo(() -> stopAll(r));
    }

    public static Command prime(RobotContainer r, double distance){
        return new InstantCommand(() -> r.shooter.distancePrime(distance));
    }

    public static Command shoot(RobotContainer r){
        return new SequentialCommandGroup(
            new InstantCommand(() -> r.gather.setGatePower(1)),
            new WaitCommand(0.2)
        );
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

    public static Translation2d getBestShootLocation(Translation2d[] shootLocations, Translation2d noteLoc, Translation2d nextLoc){
        Translation2d bestPosition = shootLocations[0];
        double bestDist = Double.MAX_VALUE; 

        for (Translation2d loc : shootLocations){
            double dist = loc.minus(noteLoc).getNorm();
            if(nextLoc != null){
                dist += nextLoc.minus(noteLoc).getNorm();
            }

            if(dist < bestDist){
                bestDist = dist;
                bestPosition = loc;
            }
        }

        return bestPosition;
    }
    
    static Thread calcThread;
    
    public static void selectedAutoThreaded(RobotContainer r, int a, int b, int c, int d, int e, int f, int g, int h, int total,
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
        double distance = Locations.tagSpeaker.minus(r.drive.getPose().getTranslation()).getNorm();
        cmd.addCommands(new SequentialCommandGroup(
            prime(r, distance),
            new WaitCommand(0.4),
            shoot(r)  
        ));
        

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

            //     A: create a path to it
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
            //TODO: could be smarter with this angle, as we can 
            //      gather from a large range of robot angles
            Translation2d vecToNote = dest.minus(startPosition);
            
            GoalEndState endState = new GoalEndState(0, vecToNote.getAngle());
            if(!Pathfinding.isNewPathAvailable()){
                callback.accept(null);
                return;
            }

            PathPlannerPath path = Pathfinding.getCurrentPath(constraints, endState);
            //     B: add a command to follow that path
            cmd.addCommands(new FollowPathHolonomic(
                path,
                r.drive::getPose,
                r.drive::getRelVelocity,
                r.drive::swerveDriveVel,
                r.drive.k.AutonPathFollowerConfig,
                r.drive.k::flipPath,
                r.drive                
            ));

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
                path = Pathfinding.getCurrentPath(constraints, endState);
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

}
