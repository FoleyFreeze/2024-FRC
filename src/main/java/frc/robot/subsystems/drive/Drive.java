package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.cals.DriveCals;
import frc.robot.subsystems.motor.Motor;

public class Drive extends SubsystemBase{
    
    RobotContainer r;
    public DriveCals k;
    SwerveDriveKinematics kinematics;
    Motor[] turnMotors;
    Motor[] driveMotors;
    AHRS navX;

    public Drive (RobotContainer r, DriveCals k){
        this.r = r;
        this.k = k;
        kinematics = new SwerveDriveKinematics(
            k.wheelCals[0].wheelLocation,
            k.wheelCals[1].wheelLocation,
            k.wheelCals[2].wheelLocation,
            k.wheelCals[3].wheelLocation
            );

        turnMotors = new Motor[4];
        driveMotors = new Motor[4];
        for (int i = 0; i<4; i++){
            driveMotors[i]  = Motor.create(k.wheelCals[i].driveMotor);
            turnMotors[i]  = Motor.create(k.wheelCals[i].swerveMotor);
        }

        navX = new AHRS(Port.kMXP);
    }
 
    public Rotation2d getRobotAngle(){
        SmartDashboard.putNumber("Yaw", navX.getYaw());
        SmartDashboard.putNumber("Angle", navX.getAngle());
        SmartDashboard.putNumber("FusedHeading", navX.getFusedHeading());
        
        return new Rotation2d(Units.degreesToRadians(-navX.getYaw()));
    }
    
    public void swerveDrive(ChassisSpeeds speeds, boolean fieldOriented){
        if(fieldOriented){
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRobotAngle());
        }
        
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, k.maxWheelSpeed);

        for(int i = 0; i < 4; i++){
            SwerveModuleState finalState = SwerveModuleState.optimize(moduleStates[i], turnMotors[i].getRotation());
            turnMotors[i].setRotation(finalState.angle);
            driveMotors[i].setPower(finalState.speedMetersPerSecond/k.maxWheelSpeed);
        }

    }

}
