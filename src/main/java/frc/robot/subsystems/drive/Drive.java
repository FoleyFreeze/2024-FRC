package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.cals.DriveCals;

public class Drive extends SubsystemBase{
    
    RobotContainer r;
    public DriveCals k;
    SwerveDriveKinematics kinematics;

    public Drive (RobotContainer r, DriveCals k){
        this.r = r;
        this.k = k;
        kinematics = new SwerveDriveKinematics(k.locFL, k.locFR, k.locBL, k.locBR);

    }

    public Rotation2d getRobotAngle(){
        return 0;//TODO: actually do this
    }
    
    public void swerveDrive(ChassisSpeeds speeds, boolean fieldOriented){
        if(fieldOriented){
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRobotAngle());
        }
        
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 0);
        
    }

}
