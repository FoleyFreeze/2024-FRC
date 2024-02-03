package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.cals.VisionCals;

public class Vision extends SubsystemBase{
    RobotContainer r;
    VisionCals k;
    VisionIO io;
    VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

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

    public Translation2d getNoteLocation(){
        //data from camera has the field in (x,z) plane and rotation around the y axis
        //we turn that into a pose2d representation 
        //distance provided is only the z component (on axis)
        //we must calculate the true distance (hypotenuse)
        //also the camera angle is +cw which is backwards compared to ours

        double dist = inputs.noteData.pose.getZ();
        double angle = inputs.noteData.pose.getRotation().getY();
        double radius = dist / Math.acos(-angle);
        Translation2d camRelNoteLoc = new Translation2d(radius, new Rotation2d(-angle));
        Logger.recordOutput("Vision/camRelNoteLoc", camRelNoteLoc);

        //camera relative -> bot relative -> field relative
        Translation2d roboRelNoteLocation = camRelNoteLoc.rotateBy(k.camLocation.getRotation()).plus(k.camLocation.getTranslation());
        Logger.recordOutput("Vision/roboRelNoteLocation", roboRelNoteLocation);
        Pose2d robotPose = posePicker(inputs.noteTimeStamp);
        Logger.recordOutput("Vision/robotPose", robotPose);
        Translation2d fieldRelNoteLocation = roboRelNoteLocation.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());

        Logger.recordOutput("Vision/fieldRelNoteLocation", fieldRelNoteLocation);
        return fieldRelNoteLocation;
    }

    public boolean hasNoteImage(){
        return inputs.now - inputs.noteTimeStamp < k.maxNoteAge;
    }

     @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
        
        updatePoseBuffer();
    }
}