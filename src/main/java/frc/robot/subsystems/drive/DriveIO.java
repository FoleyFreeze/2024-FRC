package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface DriveIO {
    @AutoLog
    public static class DriveIOInputs{
        public boolean navXconnected;
        public Rotation2d yaw = new Rotation2d();
        public Rotation2d angle = new Rotation2d();
        public Rotation2d fusedHeading = new Rotation2d();
        public Rotation2d pitch = new Rotation2d();
        public Rotation2d roll = new Rotation2d();
        public double yawVelocity = 0;
    }

    @AutoLog
    public static class FileIOInputs{
        public double[] rotationOffsets = new double[4];
    }

    public default void updateInputs(DriveIOInputs inputs) {}

    public default void updateFileInputs(FileIOInputs inputs) {}

    public default void writeOffsets(double offset[]) {}
}
