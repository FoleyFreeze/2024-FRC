package frc.robot.auton;

import java.util.List;

import javax.tools.JavaFileManager.Location;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.StartLocationType;
import frc.robot.commands.drive.CmdDriveNoteTraj;
import frc.robot.commands.gather.CmdGather;

public class CmdAuton {

    static double[] centerCloseNoteShot = {41, 6000};
    static double[] leftCloseNoteShot = {36.5, 6000};
    static double[] rightCloseNoteShot = {38.5, 6000};

    public static PathConstraints constraints = new PathConstraints(
        3.5, //vel m/s
        5,//3, //2.75, //accel m/s/s 
        3, //vel rad/s
        3  //accel rad/s/s
    );

    //if both the current and next note are close, lookup the shoot position on this table
    static int[][] shootPositionTable = {{0, 0, 0},
                                         {0, 0, 1},
                                         {0, 1, 0}};
                                                    

    static double driveToNoteThreshClose = Units.inchesToMeters(0);
    static double driveToNoteThreshFar = Units.inchesToMeters(48);
    static double driveToNoteThresh2 = Units.inchesToMeters(48);
    public static Rotation2d shooterOffset = Rotation2d.fromDegrees(4.5 + 1.25);//we shoot a bit right, so compensate left

    static boolean fastCloseNoteShots = false;
    static double fastDistToNoteThresh = Units.inchesToMeters(36);
    static double fastDistToBotThreshClose = Units.inchesToMeters(0);
    static double fastDistToBotThreshFar = Units.inchesToMeters(48);

    static boolean dodgeCloseNotes = true;//TODO: try this
    static Rotation2d forwardDir;

    public static Command selectedAuto(RobotContainer r, 
                                       int a, int b, int c, int d, int e, int f, int g, int h, int total,
                                       StartLocationType startLocation,
                                       int waitTime){

        int noteOrder[] = new int[8];
        sort(noteOrder, a, 1);
        sort(noteOrder, b, 2);
        sort(noteOrder, c, 3);
        sort(noteOrder, d, 4);
        sort(noteOrder, e, 5);
        sort(noteOrder, f, 6);
        sort(noteOrder, g, 7);
        sort(noteOrder, h, 8);
        
        SequentialCommandGroup fullSequence = new SequentialCommandGroup();

        //Step -1: wait for a bit
        //might as well save the 20ms of time when wait is zero
        if(waitTime > 0){
            fullSequence.addCommands(new WaitCommand(waitTime));
        }

        if(isBlueAlliance()){
            forwardDir = new Rotation2d();
        } else {
            forwardDir = Rotation2d.fromDegrees(180);
        }
        
        if(fastCloseNoteShots){
            fullSequence.addCommands(fastAuto(r, noteOrder, startLocation));
        } else {
            fullSequence.addCommands(slowButWorkingAuto(r, noteOrder, startLocation));
        }

        Command cmd = fullSequence.finallyDo(() -> stopAll(r));
        cmd.setName("SelectableAuto");
        return cmd;
    }

    private static boolean stateMissingNote = false;
    private static Command fastAuto(RobotContainer r, int[] noteOrder, StartLocationType startLocation){
        SequentialCommandGroup sequence = new SequentialCommandGroup();

        //Step 0: set the start position
        // and shoot first note
        sequence.addCommands(new ParallelCommandGroup(
             resetPosition(r, startLocation),
             new InstantCommand(() -> stateMissingNote = false), //reset state
             //shoot first note
             new SequentialCommandGroup(
                prime(r, startLocation),
                new WaitCommand(0.4),//TODO: how much faster before we miss
                shoot(r)
             )
        ));
        
        Pose2d startPose = getStartPose(r, startLocation);

        //for each note
        int prevNote = 0;
        Translation2d lastNoteLocation = startPose.getTranslation();
        Translation2d lastShootLocation = startPose.getTranslation();
        for(int i=0;i<noteOrder.length;i++){
            int currNote = noteOrder[i];
            int nextNote = 0;
            if(i+1 < noteOrder.length){
                nextNote = noteOrder[i+1];
            }
            if(currNote == 0){
                //no next note, we are done
                break;
            }

            //get positions
            Translation2d noteLocation = Locations.notes[currNote -1];
            Translation2d nextNoteLoc = null;
            if(nextNote > 0){
                nextNoteLoc = Locations.notes[nextNote -1];
            }
            Translation2d shootLocation = getBestShootLocation(Locations.shootingPositions, noteLocation, nextNoteLoc);

            if(currNote > 5){
                //close notes - use immediate shoot strategy
                Translation2d shootVec = noteLocation.minus(Locations.tagSpeaker); //note that the subtraction is inverted so the angle points the back of the robot at the speaker
                Rotation2d targetAngleToSpeaker = shootVec.getAngle();
                Rotation2d targetAngleForShot = targetAngleToSpeaker.plus(shooterOffset);//we shoot slightly right
                
                //extra angle for podium 
                if(isBlueAlliance() && currNote == 8){
                    targetAngleForShot = targetAngleForShot.plus(Rotation2d.fromDegrees(-3));
                } else if(!isBlueAlliance() && currNote == 6){
                    targetAngleForShot = targetAngleForShot.plus(Rotation2d.fromDegrees(3));
                }
                //shoot location is a bit less than half a robot length from the note in the speaker direction
                shootLocation = noteLocation.minus(new Translation2d(Locations.robotLength/2 - Units.inchesToMeters(6), 0).rotateBy(targetAngleToSpeaker));
                double shotDistance = shootLocation.minus(Locations.tagSpeaker).getNorm();
                Translation2d preShootLoc;

                double[] selectedShot;
                switch(currNote){
                    case 6:
                        selectedShot = leftCloseNoteShot;
                        break;
                    case 7:
                        selectedShot = centerCloseNoteShot;
                        break;
                    case 8:
                        selectedShot = rightCloseNoteShot;
                        break;
                    default:
                        selectedShot = centerCloseNoteShot;
                }

                if(isBlueAlliance()){
                    preShootLoc = shootLocation.minus(new Translation2d(Units.inchesToMeters(28), 0));
                } else {
                    preShootLoc = shootLocation.plus(new Translation2d(Units.inchesToMeters(28), 0));
                }

                //create path into note directly
                Translation2d overdrive = new Translation2d(Units.inchesToMeters(6), 0).rotateBy(forwardDir);
                List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                    new Pose2d(preShootLoc, forwardDir),
                    new Pose2d(shootLocation.plus(overdrive), forwardDir)
                );

                PathPlannerPath path = new PathPlannerPath(
                    bezierPoints, 
                    constraints, 
                    new GoalEndState(0, targetAngleForShot)
                );

                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                    path,
                    constraints
                );

                sequence.addCommands(
                    //start gather/gate/shooter so the note will pass through as fast as possible
                    new InstantCommand(() -> {r.gather.setGatherPower(0.65, 1);
                                              //r.shooter.distancePrime(shotDistance);
                                              r.shooter.commandPrime(selectedShot[0], selectedShot[1]);
                                             }, r.gather, r.shooter),
                    new ParallelRaceGroup(
                        new SequentialCommandGroup(
                            new ParallelRaceGroup(
                                pathfindingCommand,
                                //abort path command when we see a note with the camera in the right place
                                new WaitUntilCommand(() -> noteIsSeenInRange(r, noteLocation, fastDistToNoteThresh, fastDistToBotThreshClose))
                            ),
                            new PrintCommand("end pathfind begin drivenote, note: " + currNote),
                            new ParallelRaceGroup(
                                new CmdDriveNoteTraj(r, forwardDir, targetAngleForShot, 0),
                                new WaitCommand(2.5) //TODO: figure out the right time
                            ).onlyIf(() -> noteIsSeenInRange(r, noteLocation, fastDistToNoteThresh, fastDistToBotThreshClose)),
                            new WaitCommand(0.2),
                            new PrintCommand("end alldrive, note: " + currNote)
                        ),
                        new SequentialCommandGroup(
                            new WaitCommand(0.75),
                            //wait until shooter up to speed
                            new WaitUntilCommand(() -> (r.shooter.rpmSetpoint*2 - r.shooter.inputs.shootBottomVelocity - r.shooter.inputs.shootTopVelocity)
                                                        < 600),
                            //wait until shooter speed drops due to note passing through
                            new WaitUntilCommand(() -> (r.shooter.rpmSetpoint*2 - r.shooter.inputs.shootBottomVelocity - r.shooter.inputs.shootTopVelocity) 
                                                        > 1000),
                            new PrintCommand("end shot, note: " + currNote)
                        )
                    )
                );

            } else {
                //far notes, use the drive, pickup, drive, shoot strategy

                if(prevNote == 0 && startLocation == StartLocationType.AMP_SIDE_SPEAKER){
                    //if this far note is the first note, then make sure to dodge the close notes when leaving
                    
                } else {
                    //not the first note, just go to it

                }

                Translation2d overdrive = new Translation2d(Units.inchesToMeters(16),0).rotateBy(forwardDir);
                Command notePathfindingCommand = AutoBuilder.pathfindToPose(
                    new Pose2d(noteLocation.plus(overdrive), forwardDir),
                    constraints
                );
                

                Translation2d vecToNote = noteLocation.minus(lastNoteLocation);
                Command missedNotePathfindingCommand = AutoBuilder.pathfindToPose(
                    new Pose2d(noteLocation, vecToNote.getAngle()),
                    constraints
                );

                Translation2d shootVec = shootLocation.minus(Locations.tagSpeaker); //note that the subtraction is inverted so the angle points the back of the robot at the speaker
                Rotation2d targetAngleToSpeaker = shootVec.getAngle();
                Rotation2d targetAngleForShot = targetAngleToSpeaker.plus(shooterOffset);//we shoot slightly right

                Command shootPathfindingCommand = AutoBuilder.pathfindToPose(
                    new Pose2d(shootLocation, targetAngleForShot),
                    constraints
                );

                sequence.addCommands(
                    new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                            //gather logic
                            new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                    new ConditionalCommand(
                                        //go to next note gather since we missed the last one
                                        new ParallelRaceGroup(
                                            missedNotePathfindingCommand,
                                            new WaitUntilCommand(() -> noteIsSeenInRange(r, noteLocation, fastDistToNoteThresh, fastDistToBotThreshFar))
                                        ).beforeStarting(new PrintCommand("begin path, missed note, currNote: " + currNote)),
                                        //go to next note gather from shoot position
                                        new ParallelRaceGroup(
                                            notePathfindingCommand,
                                            new WaitUntilCommand(() -> noteIsSeenInRange(r, noteLocation, fastDistToNoteThresh, fastDistToBotThreshFar))
                                        ).beforeStarting(new PrintCommand("begin path, gathered note, currNote: " + currNote)),
                                        () -> stateMissingNote //did we miss the previous gather
                                    ),
                                    new PrintCommand("end pathfind, note: " + currNote),
                                    new ParallelRaceGroup(
                                        new CmdDriveNoteTraj(r, 18),
                                        new WaitCommand(1.5) 
                                    ).beforeStarting(new PrintCommand("begin drivenote, currNote: " + currNote))
                                        .onlyIf(() -> noteIsSeenInRange(r, noteLocation, fastDistToNoteThresh, fastDistToBotThreshClose)),
                                    new InstantCommand(() -> stateMissingNote = true),
                                    new WaitCommand(0.4),
                                    //if we get here, we missed the note
                                    new PrintCommand("end alldrive, note: " + currNote)
                                ),
                                new SequentialCommandGroup(
                                    new WaitCommand(0.25), //spinup time
                                    new WaitUntilCommand(() -> r.gather.inputs.intakeCurrentAmps > 12),
                                    new InstantCommand(() -> stateMissingNote = false),
                                    new PrintCommand("end gathercurrent, note: " + currNote),
                                    new WaitCommand(0.2) //bit of extra time to insure note is in robot
                                )
                            ),
                            //shoot logic
                            new SequentialCommandGroup(
                                prime(r, shootVec.getNorm()),
                                shootPathfindingCommand,
                                new PrintCommand("end shoot pathfind, note: " + currNote),
                                shoot(r)
                            ).onlyIf(() -> !stateMissingNote)
                        ),//end seq
                        CmdGather.autonGather(r).andThen(new PrintCommand("end fullgather, note: " + currNote))
                    )//end deadline
                );

            }
            
            prevNote = currNote;
            lastNoteLocation = noteLocation;
            lastShootLocation = shootLocation;
        }
        

        return sequence;
    }

    public static boolean noteIsSeenInRange(RobotContainer r, Translation2d noteLoc, double threshToNote, double threshToBot){
        if(r.vision.hasNoteImage()){
            Translation2d camNote = r.vision.getCachedNoteLocation();
            //the note is relatively close to the robot
            double distToBot = r.drive.getPose().getTranslation().minus(camNote).getNorm();
            //the note is relatively close to where it is supposed to be on the field
            double distToNote = noteLoc.minus(camNote).getNorm();
            return distToBot < threshToBot && distToNote < threshToNote;
        } else {
            //no image
            return false;
        }
    }

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
    private static Command slowButWorkingAuto(RobotContainer r, int[] noteOrder, StartLocationType startLocation){
        SequentialCommandGroup fullCommand = new SequentialCommandGroup();
        
        //Step 0: set the start position
        fullCommand.addCommands(resetPosition(r, startLocation));

        //shoot first note
        Pose2d startPose = getStartPose(r, startLocation);
        double shootDist = Locations.tagSpeaker.minus(startPose.getTranslation()).getNorm();

        fullCommand.addCommands(new SequentialCommandGroup(
            prime(r, startLocation),
            new WaitCommand(0.4),
            shoot(r)
        ));

        //for each note
        int prevNote = 0;
        for(int i=0;i<noteOrder.length;i++){
            if(noteOrder[i] == 0){
                //stop if there is no note at this position
                break;
            }
            int currNote = noteOrder[i];
            int nextNote = 0;
            if(i+1 < noteOrder.length){
                nextNote = noteOrder[i+1];
            }

            // ----------- Pathfind to Note -------------
            Translation2d noteLocation = Locations.notes[currNote - 1];
            //Translation2d vecToNote = noteLocation.minus(startPose.getTranslation());
            //point at the speaker instead of the start
            Translation2d vecToNote = noteLocation.minus(Locations.tagSpeaker);

            //offset the note 1/3 robot len in the direction we will approach from
            //Translation2d targetLocation = noteLocation.minus(new Translation2d(Locations.robotLength/5, 0).rotateBy(vecToNote.getAngle()));
            Translation2d targetLocation = noteLocation;
            if(isBlueAlliance()){
                if(currNote == 8){
                    targetLocation = targetLocation.plus(new Translation2d(0, -Units.inchesToMeters(6)));
                } else if(currNote == 6){
                    targetLocation = targetLocation.plus(new Translation2d(0, Units.inchesToMeters(6)));
                }
            } else {
                if(currNote == 6){
                    targetLocation = targetLocation.plus(new Translation2d(0, -Units.inchesToMeters(6)));
                } else if(currNote == 8){
                    targetLocation = targetLocation.plus(new Translation2d(0, Units.inchesToMeters(6)));
                }
            }
            
            Pose2d noteTargetPose = new Pose2d(targetLocation, vecToNote.getAngle().plus(shooterOffset));
            //add the shooter offset angle to the gather angle. should help be in the right orientation for the shot later

            Command pathFindingCommand;
            if(i == 0 && currNote < 6 && dodgeCloseNotes
                    && (startLocation == StartLocationType.AMP_SIDE_SPEAKER
                    || startLocation == StartLocationType.SPEAKER_CENTER)){
                //if the first note is far, and we are starting amp side
                //attempt to dodge it by going between the 2 close notes
                Translation2d inbetweenNoteLoc;
                if(isBlueAlliance()){
                    //blue side is notes 6 and 7
                    inbetweenNoteLoc = Locations.blueNoteF.plus(Locations.blueNoteG).div(2);
                } else {
                    //red side is notes 7 and 8
                    inbetweenNoteLoc = Locations.redNoteG.plus(Locations.redNoteH).div(2);
                }
                pathFindingCommand = AutoBuilder.pathfindToPose(
                    new Pose2d(inbetweenNoteLoc, forwardDir),
                    constraints,
                    constraints.getMaxVelocityMps(), 0.0
                ).andThen(AutoBuilder.pathfindToPose(
                    noteTargetPose,
                    constraints,
                    0.0, 0.0
                )).finallyDo(() -> r.drive.swerveDrivePwr(new ChassisSpeeds()));

            } else {
                pathFindingCommand = AutoBuilder.pathfindToPose(
                    noteTargetPose,
                    constraints,
                    0.0, 0.0
                ).finallyDo(() -> r.drive.swerveDrivePwr(new ChassisSpeeds()));
            }

            //if close notes
            double driveToNoteThresh;
            if(currNote > 5){
                driveToNoteThresh = driveToNoteThreshClose;
            } else {
                driveToNoteThresh = driveToNoteThreshFar;
            }

            //construct the complex pathfind to note and gather it with vision command
            Command noteCommand;

            if(currNote < 6){
                //use the right pickup angle for the far notes
                pathFindingCommand = pathFindingCommand.deadlineWith(setRotationOverride(r, noteTargetPose));
            }

            noteCommand = pathFindingCommand
                //until we are close to the note (and vision sees a note that close)
                .until(() -> {var botLoc = r.drive.getPose().getTranslation();
                        return botLoc.minus(noteLocation).getNorm() < driveToNoteThresh
                            && r.vision.hasNoteImage() 
                            && botLoc.minus(r.vision.getCachedNoteLocation()).getNorm() < driveToNoteThresh;}) 
                .andThen(new CmdDriveNoteTraj(r).onlyIf(() -> r.vision.hasNoteImage() 
                                                           && r.drive.getPose().getTranslation().minus(r.vision.getCachedNoteLocation()).getNorm() < driveToNoteThresh2)
                    .raceWith(new WaitCommand(1))) //give it 4 seconds before moving on to the next note
                .andThen(new WaitCommand(0.5))
            .raceWith(CmdGather.autonGather(r));

            fullCommand.addCommands(noteCommand);

            
            // ----------- Pathfind to Shoot -------------

            //determine best shooting location
            Translation2d nextNoteLoc = null;
            if(nextNote > 0){
                nextNoteLoc = Locations.notes[nextNote - 1];
            }
            //dont forget that the BACK of the robot needs to face the speaker
            System.out.println("find best shoot loc for note " + currNote);
            Translation2d shootLoc;
            boolean shootAtNoteLoc = false;
            if(currNote > 5){
                //if this note and the next are close, use the table for the best shoot spot 
                if(nextNote < 6){
                    //if the next note doesnt exist or is deep, shoot where you are
                    shootLoc = noteLocation;
                    shootAtNoteLoc = true;
                    
                    //Forget this offset for now, the same position should work since it should be speaker aligned now
                    //Translation2d toSpeaker = Locations.tagSpeaker.minus(shootLoc);
                    //since we need to move to turn, move a bit towards the speaker
                    //shootLoc = noteLocation.plus(new Translation2d(Units.inchesToMeters(6), toSpeaker.getAngle()));
                } else {
                    int shootIdx = shootPositionTable[currNote-6][nextNote-6];
                    shootLoc = Locations.shootingPositions[shootIdx];
                    System.out.println("picked shoot position " + shootIdx);
                }
            } else{
                //currNote is far
                shootLoc = getBestShootLocation(Locations.shootingPositions, noteLocation, nextNoteLoc);

                //override the speaker side notes to use the easiest shoot position
                if(isBlueAlliance()){
                    //blue
                    if(currNote == 1 || currNote == 2){
                        shootLoc = Locations.shootingPositions[2];
                    }
                } else {
                    //red
                    if(currNote == 4 || currNote == 5){
                        shootLoc = Locations.shootingPositions[2];
                    }
                }
                
            }
            Translation2d vecToSpeaker = Locations.tagSpeaker.minus(shootLoc);
            Rotation2d targetAngle = vecToSpeaker.getAngle().plus(Rotation2d.fromDegrees(180));

            Translation2d offsetDrive = new Translation2d();
            /*
            if(shootLoc == Locations.redShootingPositions[4]){
                System.out.println("offsetting red stage shoot position for note: " + currNote);
                offsetDrive = new Translation2d(-Units.inchesToMeters(3), Units.inchesToMeters(6));
            } else if(shootLoc == Locations.blueShootingPositions[4]){
                System.out.println("offsetting blue stage shoot position for note: " + currNote);
                offsetDrive = new Translation2d(Units.inchesToMeters(3), Units.inchesToMeters(6));
            }
            */
            Pose2d shotTargetPose = new Pose2d(shootLoc.plus(offsetDrive), targetAngle.plus(shooterOffset));

            double extraShootDist = 0;
            if(shootAtNoteLoc){
                //offset back for the note shots so the angle is lower 
                extraShootDist = Units.inchesToMeters(24);
                System.out.println("offsetting shoot dist 18in for note: " + currNote);
            } else if(shootLoc == Locations.shootingPositions[2]){
                extraShootDist = Units.inchesToMeters(9);
                System.out.println("offsetting shoot dist 18in for note: " + currNote);
            }
            
            pathFindingCommand = AutoBuilder.pathfindToPose(
                shotTargetPose,
                constraints,
                0.0, 0.0
            ).finallyDo(() -> r.drive.swerveDrivePwr(new ChassisSpeeds()));

            Command shootCommand = new SequentialCommandGroup(
                prime(r, vecToSpeaker.getNorm() + extraShootDist),
                pathFindingCommand,
                shoot(r)
            );

            //only run the shoot sequence if we successfully gathered a note
            //TODO: add this back if we can fix the robot direction
            fullCommand.addCommands(shootCommand/*.onlyIf(() -> r.state.hasNote)*/);

            prevNote = currNote;
        }

        return fullCommand;
    }

    public static Command setRotationOverride(RobotContainer r, Pose2d goal){
        return new RunCommand(() -> {
                                    int size = r.drive.fullPath.size();
                                    if(size > 0){
                                        int lastIdx = size - 1;
                                        if(r.drive.fullPath.get(lastIdx).getTranslation().getDistance(goal.getTranslation()) < Units.inchesToMeters(6)){
                                            Translation2d endPos = r.drive.fullPath.get(lastIdx).getTranslation();
                                            int midIdx = lastIdx - size / 5;
                                            Translation2d midPos = r.drive.fullPath.get(midIdx).getTranslation();
                                            Rotation2d targetAngle = endPos.minus(midPos).getAngle();
                                            RotationOverride.setTargetRotation(targetAngle);
                                            Logger.recordOutput("PathPlanner/RotationOverride", targetAngle);
                                            Logger.recordOutput("PathPlanner/RotationOverrideEn", true);
                                        }
                                    }
                                    })
                        .finallyDo(() -> {
                            RotationOverride.disableRotation();
                            Logger.recordOutput("PathPlanner/RotationOverrideEn", false);
                        });
    }

    public static Command prime(RobotContainer r, double distance){
        return new InstantCommand(() -> r.shooter.distancePrime(distance));
    }

    public static Command prime(RobotContainer r, StartLocationType start){
        if(DriverStation.getAlliance().isPresent()){
            if(DriverStation.getAlliance().get() == Alliance.Blue){
                if(start == StartLocationType.AMP_SIDE_SPEAKER){
                    return new InstantCommand(() -> r.shooter.commandPrime(59, 5500));
                }
            } else {
                if(start == StartLocationType.SOURCE_SIDE_SPEAKER){
                    return new InstantCommand(() -> r.shooter.commandPrime(59, 5500));
                }
            }
        }
        if(start == StartLocationType.SPEAKER_CENTER){
            return new InstantCommand(() -> r.shooter.commandPrime(57, 5500));
        } else {
            return new InstantCommand(() -> r.shooter.commandPrime(55, 5500));
        }
    }

    public static Command shoot(RobotContainer r){
        return new SequentialCommandGroup(
            new InstantCommand(() -> r.gather.setGatePower(1)),
            new WaitCommand(0.2),
            new InstantCommand(() -> r.state.hasNote = false)
        );
    }

    private static void sort(int noteOrder[], int index, int note){
        if(index>0){
            noteOrder[index - 1] = note;
        }
    }

    public static Command resetPosition(RobotContainer r, StartLocationType start){
        switch(start){
            case AMP_SIDE_SPEAKER:
            case SOURCE_SIDE_SPEAKER:
            case SPEAKER_CENTER:
            case AMP:
            case SOURCE:
                return new InstantCommand(() -> r.drive.resetFieldOdometry(Locations.startLocations[start.ordinal()]));
            case APRILTAG_0Deg:
                return new InstantCommand(() -> r.drive.resetFieldOrientedAngle(new Rotation2d()));
            case APRILTAG:
            default:
                return new InstantCommand();
        }
    }

    public static Pose2d  getStartPose(RobotContainer r, StartLocationType start){
        switch(start){
            case AMP_SIDE_SPEAKER:
            case SOURCE_SIDE_SPEAKER:
            case SPEAKER_CENTER:
            case AMP:
            case SOURCE:
                return Locations.startLocations[start.ordinal()];
            case APRILTAG_0Deg:
            case APRILTAG:
            default:
                return r.drive.getPose();
        }
    }


    public static void stopAll(RobotContainer r){
        r.shooter.goHome();
        r.gather.setGatherPower(0, 0);
    }

    public static Translation2d getBestShootLocation(Translation2d[] shootLocations, Translation2d noteLoc, Translation2d nextLoc){
        Translation2d bestPosition = shootLocations[0];
        double bestDist = Double.MAX_VALUE; 
        int count = 0;
        int bestIdx = 0;

        for (Translation2d loc : shootLocations){
            double dist = loc.minus(noteLoc).getNorm();
            if(nextLoc != null){
                dist += loc.minus(nextLoc).getNorm();
            }

            if(dist < bestDist){
                bestDist = dist;
                bestPosition = loc;
                bestIdx = count;
            }
            count++;
        }

        System.out.println("picked shoot position " + bestIdx);
        return bestPosition;
    }

    public static boolean isBlueAlliance(){
        if(DriverStation.getAlliance().isPresent()){
            if(DriverStation.getAlliance().get() == Alliance.Blue){
                return true;
            } else {
                return false;
            }
        }
        return true;
    }
    
    /*
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
    */
}
