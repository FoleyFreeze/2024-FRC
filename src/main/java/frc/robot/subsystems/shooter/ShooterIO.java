package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {

    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setShooterVoltage(double voltage) {}

    public default void setAngle(double angle) {}
}
