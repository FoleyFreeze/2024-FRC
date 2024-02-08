package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double shootTopPosition;
        public double shootTopVelocity;
        public double shootTopAppliedVolts;
        public double shootTopCurrentAmps;

        public double shootBottomPosition;
        public double shootBottomVelocity;
        public double shootBottomAppliedVolts;
        public double shootBottomCurrentAmps;

        public double anglePosition;
        public double angleVelocity;
        public double angleAppliedVolts;
        public double angleCurrentAmps;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setShooterVoltage(double voltage) {}

    public default void setAngle(double angle) {}
}
