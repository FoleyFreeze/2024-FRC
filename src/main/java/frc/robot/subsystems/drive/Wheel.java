package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.cals.DriveCals.WheelCal;

public class Wheel {

    WheelCal k;

    private final WheelIO io;
    private final WheelIOInputsAutoLogged inputs = new WheelIOInputsAutoLogged();

    SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.03, 0.13);
    PIDController driveFB = new PIDController(0.00, 0, 0);
    

    public Wheel(WheelCal k){
        this.k = k;
      //io = Robot.isReal() ? new WheelIO_HW(k) : new WheelIO() {};
      //Another way to do if 
        if (Robot.isReal() && !k.disable){
            io = new WheelIO_HW(k);
        } else {
            io = new WheelIO() {};
        }
    }

    public void updateInputs() {
        io.updateInputs(inputs);
    }

    public void periodic(){
        Logger.processInputs("Drive/Wheel" + k.name, inputs);
    }

    public SwerveModulePosition getPosition(){
        Rotation2d angle = inputs.swervePosition;
        double driveEnc = inputs.drivePosition;
        
        return new SwerveModulePosition(driveEnc, angle);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(inputs.driveVelocity, inputs.swervePosition);
    }

    public SwerveModuleState moveWheel(SwerveModuleState state, boolean stopped, boolean isVelocity){
        var optimizedState = SwerveModuleState.optimize(state, inputs.swervePosition);
        if(stopped){
            io.setDriveVoltage(0);
        } else if(isVelocity && k.driveWithVel){
            double velRadPerSec = optimizedState.speedMetersPerSecond / k.wheelRadius;
            io.setDriveVoltage(driveFF.calculate(velRadPerSec) + driveFB.calculate(Units.rotationsPerMinuteToRadiansPerSecond(inputs.driveVelocity), velRadPerSec));
            io.setSwerveAngle(optimizedState.angle);
        } else {
            io.setDriveVoltage(12 * optimizedState.speedMetersPerSecond / k.maxSpeed);
            io.setSwerveAngle(optimizedState.angle);
        }
        return optimizedState;
    }

    public void setSwerveOffset(Rotation2d offset){
        io.setSwerveOffset(offset);
    }

    public Rotation2d getAnalogEncoderValue () {
        return inputs.analogEncoderAngleRaw;
    }

    public void setBrake(boolean on){
        io.setDriveBrakemode(on);
    }
}
