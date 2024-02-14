package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.cals.DriveCals;

public class Drive extends SubsystemBase{
    
    RobotContainer r;
    public DriveCals k;

    DriveIO driveIO;
    public DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    FileIOInputsAutoLogged fileInputs = new FileIOInputsAutoLogged();

    Wheel[] wheels;
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;

    public Pose2d robotPose = new Pose2d();
    public Rotation2d robotAngle = new Rotation2d();

    Rotation2d fieldOffsetAngle = new Rotation2d();
    

    public Drive (RobotContainer r, DriveCals k){
        this.r = r;
        this.k = k;

        wheels = new Wheel[4];
    
        if(Robot.isReal() && !k.disable){
            driveIO = new DriveIO_HW();
        }else{
            driveIO = new DriveIO() {};
        }

        for (int i = 0; i<4; i++){
            wheels[i] = new Wheel(k.wheelCals[i]);
        }

        driveIO.updateFileInputs(fileInputs);
        Logger.processInputs("Drive", fileInputs);

        for(int i = 0; i<4; i++){
            wheels[i].setSwerveOffset(new Rotation2d(fileInputs.rotationOffsets[i]));
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
    }

    public void swerveDrivePwr(ChassisSpeeds speeds, boolean fieldOriented){
        boolean stopped = Math.abs(speeds.vxMetersPerSecond) < 0.02
                        && Math.abs(speeds.vyMetersPerSecond) < 0.02
                        && Math.abs(speeds.omegaRadiansPerSecond) < 0.02;
        
        //scale to m/s
        speeds = speeds.times(k.maxWheelSpeed);
        
        if(fieldOriented){
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAngle());
            speeds = ChassisSpeeds.discretize(speeds, 0.02);
        }
        
        SwerveModuleState[] wheelStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(wheelStates, k.maxWheelSpeed);

        SwerveModuleState[]  optimizedWheelStates = new SwerveModuleState[4]; 

        for(int i = 0; i < 4; i++){
            optimizedWheelStates[i] = wheels[i].moveWheel(wheelStates[i], stopped);
        }

        Logger.recordOutput("Drive/SwerveSetpoints", wheelStates);
        Logger.recordOutput("Drive/SwerveSetpointsOptimized", optimizedWheelStates);    
    }

    //return the error magnitude
    public double driveToLocation(Translation2d location){
        Translation2d error = location.minus(getPose().getTranslation());
        Logger.recordOutput("Drive/LocationError", error);

        Translation2d power = new Translation2d(k.autoGatherPower, error.getAngle());

        ChassisSpeeds speeds = new ChassisSpeeds(power.getX(), power.getY(), 0);
        swerveDrivePwr(speeds, true);

        return error.getNorm();
    }
    

    @Override
    public void periodic(){
        driveIO.updateInputs(inputs);
        for(var wheel:wheels){
            wheel.updateInputs();
        }
        Logger.processInputs("Drive", inputs);
        for(var wheel:wheels){
            wheel.periodic();
        }

        //update angle
        robotAngle = inputs.yaw.minus(fieldOffsetAngle);

        //update odometry
        robotPose = odometry.update(
            getAngle(), 
            getWheelPositions()
        );

        
    }

    @AutoLogOutput(key = "Drive/RobotPose")
    public Pose2d getPose(){
        return robotPose;
    }

    @AutoLogOutput(key = "Drive/RobotAngle")
    public Rotation2d getAngle(){
        return robotAngle;
    }

    public void resetFieldOrientedAngle(){
        fieldOffsetAngle = inputs.yaw;
        robotAngle = new Rotation2d();//reset to 0
    }

    public void resetFieldOdometry(){
        odometry.resetPosition(getAngle(), getWheelPositions(), new Pose2d(0, 0, getAngle()));
    }

    private SwerveModulePosition[] getWheelPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = wheels[i].getPosition();
        }
        return positions;
    }

    @AutoLogOutput(key = "Drive/MeasuredSwerveStates")
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
        driveIO.writeOffsets(offsets);
        System.out.println("Done!");
    }
}