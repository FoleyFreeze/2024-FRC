package frc.robot.auton;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;

public class RotationOverride {
    
    static Optional<Rotation2d> targetRotation = Optional.of(new Rotation2d());
    static boolean rotationEnabled = false;

    public static void setTargetRotation(Rotation2d r){
        targetRotation = Optional.of(r);
        rotationEnabled = true;
    }

    public static void enableRotation(boolean enable){
        rotationEnabled = enable;
    }

    public static void disableRotation(){
        enableRotation(false);
    }

    public static Optional<Rotation2d> getRotationTargetOverride(){
        if(rotationEnabled){
            return targetRotation;
        } else {
            return Optional.empty();
        }
    }

}
