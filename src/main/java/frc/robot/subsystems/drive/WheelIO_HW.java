package frc.robot.subsystems.drive;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.cals.DriveCals.WheelCal;
import frc.robot.subsystems.motor.Motor;

public class WheelIO_HW implements WheelIO {
    
    Motor driveMotor;
    Motor swerveMotor;
    AnalogInput swerveAbsoluteEncoder;
    Rotation2d swerveOffset;

    WheelCal k;

    public WheelIO_HW (WheelCal k){
        this.k = k;
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

        inputs.swervePositionRaw = swerveMotor.getRotation();
        inputs.swervePosition = inputs.swervePositionRaw.minus(swerveOffset);
        inputs.swerveVelocity = swerveMotor.getVelocity();
        inputs.swerveCurrent = driveMotor.getCurrent();
        inputs.swerveVoltage = driveMotor.getVoltage();

        inputs.analogEncoderAngleRaw = new Rotation2d(swerveAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI);
        inputs.analogEncoderAngle = inputs.analogEncoderAngleRaw.minus(swerveOffset); 
    }

    @Override
    public void setDriveVoltage(double voltage){
        driveMotor.setVoltage(voltage);
    }

    @Override
    public void setSwerveAngle(Rotation2d angle){
        swerveMotor.setRotation(angle.plus(swerveOffset));
    }

    @Override
    public void setDriveBrakemode(boolean enable){
        driveMotor.setBrakeMode(enable);
    }

    @Override
    public void setSwerveOffset(Rotation2d analogOffset){
        Rotation2d rawAnalogEncoder = new Rotation2d(swerveAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI);
        Rotation2d deltaToSavedAnalog = rawAnalogEncoder.minus(analogOffset);
        swerveOffset = swerveMotor.getRotation().minus(deltaToSavedAnalog);

        Logger.recordOutput("Drive/Offset/AnalogOffsetToZero" + k.name, analogOffset.getRadians());
        Logger.recordOutput("Drive/Offset/AnalogOffset" + k.name, deltaToSavedAnalog.getRadians());
        Logger.recordOutput("Drive/Offset/NeoEncOffset" + k.name, swerveOffset.getRadians());
    }
}
