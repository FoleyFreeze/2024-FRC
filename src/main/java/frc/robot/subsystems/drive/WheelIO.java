package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface WheelIO {
    @AutoLog
    public static class WheelIOInputs{
        public double drivePosition;
        public double driveVelocity;
        public double driveAppliedVolts;
        public double driveCurrentAmps;

        public Rotation2d swervePosition;
        public double swerveVelocity;
        public double swerveVoltage;
        public double swerveCurrent;
        
        public Rotation2d swerveEncoderPosition;
    }

    public default void updateInputs(WheelIOInputs inputs) {}

    public default void setDriveVoltage(double voltage){}

    public default void setSwerveAngle(Rotation2d angle){}

    public default void setDriveBrakemode(boolean enable) {}

    public default void setSwerveOffset(double offset) {}
}
