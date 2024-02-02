package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.cals.VisionCals;

public class Vision extends SubsystemBase{
    RobotContainer r;
    VisionCals k;
    VisionIO io;
    VisionIOInputsAutoLogged inputs;

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
        //if the time is before evything in the buffer return the oldest thing
        return robotPoseBuffer.getLast().pose;
    }

     @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
        
        updatePoseBuffer();
    }

}