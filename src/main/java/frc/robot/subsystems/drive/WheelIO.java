package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface WheelIO {
    @AutoLog
    public static class WheelIOInputs{
        public double drivePosition;
        public double driveVelocity;
        public double driveAppliedVolts;
    }

}
