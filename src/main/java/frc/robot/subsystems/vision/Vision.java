package frc.robot.subsystems.vision;

import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.auton.Locations;
import frc.robot.cals.VisionCals;

public class Vision extends SubsystemBase{
    RobotContainer r;
    public VisionCals k;
    VisionIO io;
    VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    Translation2d lastNoteLocation;

    int badIdErr = 0;
    int badHeightErr = 0;
    int badXErr = 0;

    DoubleEntry rioTime = NetworkTableInstance.getDefault().getDoubleTopic("/Vision/RIO Time").getEntry(0);

    public class TimestampedPose2d{
        Pose2d pose;
        double time;
    }

    CircularBuffer<TimestampedPose2d> robotPoseBuffer;

    public Vision(RobotContainer r, VisionCals k){
        this.r = r;
        this.k = k;

        if(Robot.isReal() && !k.disable){
            io = new VisionIO_HW();
        }else{
            io = new VisionIO(){};
        }

        robotPoseBuffer = new CircularBuffer<>(k.bufferSize);
    }

    private void updatePoseBuffer(){
        TimestampedPose2d now  = new TimestampedPose2d();
        now.pose = r.drive.getPose();
        now.time = inputs.now;
        robotPoseBuffer.addFirst(now);
    }

    private Pose2d posePicker(double time){
        TimestampedPose2d prev = robotPoseBuffer.getFirst(); 
        for(int i = 0; i < robotPoseBuffer.size(); i++){
            TimestampedPose2d next = robotPoseBuffer.get(i);
            double delta = next.time - time;
            if(delta < 0){
                double t =  ((time - next.time) / (prev.time - next.time));
                return next.pose.interpolate(prev.pose, t);
            }
        }
        //if the time is before everything in the buffer return the oldest thing
        return robotPoseBuffer.getLast().pose;
    }

    public Translation2d calcNoteLocation(){
        //data from camera has the field in (x,z) plane and rotation around the y axis
        //we turn that into a pose2d representation 
        //distance provided is only the z component (on axis)
        //we must calculate the true distance (hypotenuse)
        //also the camera angle is +cw which is backwards compared to ours

        double dist = Units.inchesToMeters(inputs.noteData.distance);
        double angle = -inputs.noteData.angle;
        double radius = dist / Math.cos(angle);
        Translation2d camRelNoteLoc = new Translation2d(radius, new Rotation2d(angle));
        Logger.recordOutput("Vision/camRelNoteLoc", camRelNoteLoc);

        //camera relative -> bot relative -> field relative
        Translation2d roboRelNoteLocation = camRelNoteLoc.rotateBy(k.camLocation.getRotation()).plus(k.camLocation.getTranslation());
        Logger.recordOutput("Vision/roboRelNoteLocation", roboRelNoteLocation);
        Pose2d robotPose = posePicker(inputs.noteData.timeStamp);
        Logger.recordOutput("Vision/robotPose", robotPose);
        Translation2d fieldRelNoteLocation = roboRelNoteLocation.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());

        //done in periodic
        //Logger.recordOutput("Vision/fieldRelNoteLocation", fieldRelNoteLocation);

        //mark as processed
        inputs.noteData.isProcessed = true;

        return fieldRelNoteLocation;
    }

    public Translation2d getCachedNoteLocation(){
        return lastNoteLocation;
    }

    public boolean hasNoteImage(){
        return inputs.now - inputs.noteData.timeStamp < k.maxNoteAge;
    }

    public boolean hasNewNoteImage(){
        return !inputs.noteData.isProcessed && hasNoteImage();
    }

    public boolean hasTagImage(){
        return inputs.now - inputs.tagData.timestamp < k.maxTagAge;
    }

    public boolean hasNewTagImage(){
        return !inputs.tagData.isProcessed && hasTagImage();
    }

    static final Translation2d zeroT2D = new Translation2d();

     @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);

        updatePoseBuffer();

        rioTime.set(Logger.getRealTimestamp() / 1000000.0);

        if(hasNewNoteImage()){
            lastNoteLocation = calcNoteLocation();
            Logger.recordOutput("Vision/fieldRelNoteLocation", lastNoteLocation);
        } else {
            //Logger.recordOutput("Vision/fieldRelNoteLocation", zeroT2D);
        }
        

        if(hasNewTagImage()){
            for(int i=0;i<inputs.tagData.tagCount;i++){
                VisionTag tag = inputs.tagData.tags[i];

                //create pose3d
                Pose3d rawTagPose = new Pose3d(new Translation3d(tag.transX, tag.transY, tag.transZ), new Rotation3d(tag.rotX, tag.rotY, tag.rotZ));
                Logger.recordOutput("Vision/RawTagPose", rawTagPose);
                Logger.recordOutput("Vision/RawTag/RotX", Math.toDegrees(rawTagPose.getRotation().getX()));
                Logger.recordOutput("Vision/RawTag/RotY", Math.toDegrees(rawTagPose.getRotation().getY()));
                Logger.recordOutput("Vision/RawTag/RotZ", Math.toDegrees(rawTagPose.getRotation().getZ()));

                Pose3d tempPose = rawTagPose.rotateBy(k.tagCamLocation.getRotation());
                Pose3d rawRobotPose = new Pose3d(tempPose.getTranslation().plus(k.tagCamLocation.getTranslation()), tempPose.getRotation());
                Logger.recordOutput("Vision/RawRobotTagPose", rawRobotPose);
                Logger.recordOutput("Vision/RawRobotTag/RotX", Math.toDegrees(rawRobotPose.getRotation().getX()));
                Logger.recordOutput("Vision/RawRobotTag/RotY", Math.toDegrees(rawRobotPose.getRotation().getY()));
                Logger.recordOutput("Vision/RawRobotTag/RotZ", Math.toDegrees(rawRobotPose.getRotation().getZ()));

                /*
                Pose2d robotPose = posePicker(inputs.tagData.timestamp);
                Pose3d botFieldPose = new Pose3d(new Translation3d(robotPose.getX(), robotPose.getY(), 0), new Rotation3d(0, 0, robotPose.getRotation().getRadians()));
                tempPose = rawRobotPose.rotateBy(botFieldPose.getRotation());
                Pose3d tagFieldPose = new Pose3d(tempPose.getTranslation().plus(botFieldPose.getTranslation()), tempPose.getRotation());
                Logger.recordOutput("Vision/FieldTagPose", tagFieldPose);
                Logger.recordOutput("Vision/FieldTag/RotX", Math.toDegrees(tagFieldPose.getRotation().getX()));
                Logger.recordOutput("Vision/FieldTag/RotY", Math.toDegrees(tagFieldPose.getRotation().getY()));
                Logger.recordOutput("Vision/FieldTag/RotZ", Math.toDegrees(tagFieldPose.getRotation().getZ()));
                */
                //dont do it the above way, instead use where the tag should be, and offset that by the camera location to get the robot pose according to the camera

                
                //first check height vs reality to reject incorrect heights
                Optional<Pose3d> fieldTagPose = Locations.tagLayout.getTagPose(tag.tagId);
                if(!fieldTagPose.isPresent()) {
                    if(k.printDebugTagData) System.out.println("Tag: " + tag.tagId + " does not exist");
                    badIdErr++;
                    Logger.recordOutput("Vision/Error/BadIdCount", badIdErr);
                    Logger.recordOutput("Vision/Error/LastBadId", tag.tagId);
                    break;
                } 

                double targetHeight = fieldTagPose.get().getZ();
                if(Math.abs(targetHeight - rawRobotPose.getZ()) > Units.inchesToMeters(8)){
                    if(k.printDebugTagData) System.out.println("Tag height doesn't match the field id:" + tag.tagId + " height: " + Units.metersToInches(rawRobotPose.getZ()));
                    badHeightErr++;
                    Logger.recordOutput("Vision/Error/BadHeightCount", badHeightErr);
                    Logger.recordOutput("Vision/Error/LastBadHeightDelta", Math.abs(targetHeight - rawRobotPose.getZ()));
                    break;
                }

                if(tag.transX < 0){
                    if(k.printDebugTagData) System.out.println("Tag is located behind the camera?!? id: " + tag.tagId + " x: " + Units.metersToInches(tag.transX));
                    badXErr++;
                    Logger.recordOutput("Vision/Error/BadDistCount", badXErr);
                    Logger.recordOutput("Vision/Error/LastBadDist", tag.transX);
                    break;
                }

                //get the camera's version of the robot pose
                tempPose = rawRobotPose.rotateBy(new Rotation3d(0, 0, r.drive.getAngle().getRadians()));
                Pose3d robotPose = new Pose3d(fieldTagPose.get().getTranslation().minus(tempPose.getTranslation()), tempPose.getRotation());
                Logger.recordOutput("Vision/RobotEstPose", robotPose);
                Logger.recordOutput("Vision/RobotEst/RotX", Math.toDegrees(robotPose.getRotation().getX()));
                Logger.recordOutput("Vision/RobotEst/RotY", Math.toDegrees(robotPose.getRotation().getY()));
                Logger.recordOutput("Vision/RobotEst/RotZ", Math.toDegrees(robotPose.getRotation().getZ()));

                
                //eventually update odometry
                //r.drive.odometry.addVisionMeasurement(new Pose2d(robotPose.getTranslation().toTranslation2d(), r.drive.getAngle()), 
                //                                        inputs.tagData.timestamp, 
                //                                        VecBuilder.fill(1, 1, 1));

            }

            inputs.tagData.isProcessed = true;

        }
    }
}