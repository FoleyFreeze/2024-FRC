package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.cals.DriveCals;
import frc.robot.subsystems.drive.DriveIO.FileIOInputs;

public class Drive extends SubsystemBase{
    
    RobotContainer r;
    public DriveCals k;

    DriveIO driveIO;
    DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    FileIOInputsAutoLogged fileInputs = new FileIOInputsAutoLogged();
    public Pose2d robotPose = new Pose2d();

    Wheel[] wheels;
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    

    public Drive (RobotContainer r, DriveCals k){
        this.r = r;
        this.k = k;

        wheels = new Wheel[4];
    
        if(Robot.isReal()){
            driveIO = new DriveIO_HW();
        }else{
            driveIO = new DriveIO() {};
        }

        for (int i = 0; i<4; i++){
            wheels[i] = new Wheel(k.wheelCals[i]);
        }

        kinematics = new SwerveDriveKinematics(
            k.wheelCals[0].wheelLocation,
            k.wheelCals[1].wheelLocation,
            k.wheelCals[2].wheelLocation,
            k.wheelCals[3].wheelLocation
            );

        odometry = new SwerveDriveOdometry(
            kinematics, 
            new Rotation2d(),
            getWheelPositions()
        );

        driveIO.updateFileInputs(fileInputs);

        for(int i = 0; i<4; i++){
            wheels[i].setSwerveOffset(new Rotation2d(fileInputs.rotationOffsets[i]));
        }
    }
 
   
    public void swerveDrivePwr(ChassisSpeeds speeds, boolean fieldOriented){
        boolean stopped = Math.abs(speeds.vxMetersPerSecond) < 0.02
                        && Math.abs(speeds.vyMetersPerSecond) < 0.02
                        && Math.abs(speeds.omegaRadiansPerSecond) < 0.02;
        
        //scale to m/s
        speeds = speeds.times(k.maxWheelSpeed);
        
        if(fieldOriented){
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, inputs.yaw);
            speeds = ChassisSpeeds.discretize(speeds, 0.02);
        }
        
        SwerveModuleState[] wheelStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(wheelStates, k.maxWheelSpeed);

        SwerveModuleState[]  optimizedWheelStates = new SwerveModuleState[4]; 
        for(int i = 0; i < 4; i++){
           optimizedWheelStates[i] = wheels[i].moveWheel(wheelStates[i]);
        }

        Logger.recordOutput("SwerveStates/setpoints", wheelStates);
        Logger.recordOutput("SwerveStates/SetpoinntsOptimized", optimizedWheelStates);    
    }

    @Override
    public void periodic(){
        driveIO.updateInputs(inputs);
        for(var wheel:wheels){
            wheel.updateInputs();
        }
        Logger.processInputs("Drive/Gyro", inputs);
        for(var wheel:wheels){
            wheel.periodic();
        }

        //update odometry
        robotPose = odometry.update(
            inputs.yaw, 
            getWheelPositions()
        );

        
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose(){
        return robotPose;
    }

    private SwerveModulePosition[] getWheelPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = wheels[i].getPosition();
        }
        return positions;
    }

    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getWheelStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = wheels[i].getState();
        }
        return states;
    }

    //get offset for every wheels, apply offset, write to file
    public void learnSwerveOffsets(){
        System.out.println("Saving new Wheel Offsets...");
        double offsets[] = new double[4];
        for (int i = 0; i < 4; i++) {
            Rotation2d rotation = wheels[i].getAnalogEncoderValue();
            offsets[i] = rotation.getRadians();
            wheels[i].setSwerveOffset(rotation);
        }
        System.out.println("Done!");
    }

}