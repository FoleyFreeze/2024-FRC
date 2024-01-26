package frc.robot.subsystems.drive;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.cals.DriveCals.WheelCal;
import frc.robot.subsystems.motor.Motor;

public class WheelIO_HW implements WheelIO {
    
    Motor driveMotor;
    Motor swerveMotor;
    AnalogInput swerveAbsoluteEncoder;
    Rotation2d absoluteEncoderOffset;

    public WheelIO_HW (WheelCal k){
        driveMotor = Motor.create(k.driveMotor);
        swerveMotor = Motor.create(k.swerveMotor);
        swerveAbsoluteEncoder = new AnalogInput(k.encChannel);
    }

    @Override
    public void updateInputs (WheelIOInputs inputs){
        inputs.drivePosition = driveMotor.getPosition();
        inputs.driveVelocity = driveMotor.getVelocity();
        inputs.driveCurrentAmps = driveMotor.getCurrent();
        inputs.driveAppliedVolts = driveMotor.getVoltage();

        inputs.swervePosition = swerveMotor.getRotation().minus(absoluteEncoderOffset);
        inputs.swerveVelocity = swerveMotor.getVelocity();
        inputs.swerveCurrent = driveMotor.getCurrent();
        inputs.swerveVoltage = driveMotor.getVoltage();

        inputs.swerveEncoderPosition = new Rotation2d(
                swerveAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI)
                .minus(absoluteEncoderOffset);
    }

    @Override
    public void setDriveVoltage(double voltage){
        driveMotor.setVoltage(voltage);
    }

    @Override
    public void setSwerveAngle(Rotation2d angle){
        swerveMotor.setRotation(angle.plus(absoluteEncoderOffset));
    }

    @Override
    public void setDriveBrakemode(boolean enable){
        driveMotor.setBrakeMode(enable);
    }

    @Override
    public void setSwerveOffset(double offset){
        absoluteEncoderOffset = new Rotation2d(offset);
    }
}
