package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
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
    SwerveDrivePoseEstimator odometry;

    public Pose2d robotPose = new Pose2d();
    public ChassisSpeeds robotRelVelocity = new ChassisSpeeds();
    

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

        odometry = new SwerveDrivePoseEstimator(
            kinematics, 
            new Rotation2d(),
            getWheelPositions(),
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.9, 0.9, 0.9)
        );
    }

    public void swerveDrivePwr(ChassisSpeeds speeds){
        ChassisSpeeds power = speeds.div(k.maxWheelSpeed);
        limitSpeeds(power, k.autoDrivePower);
        
        swerveDrivePwr(power, false);
    }

    public void swerveDriveVel(ChassisSpeeds speeds){
        //use as velocity this time
        swerveDrivePwr(speeds, false, true);
    }

    public void swerveDrivePwr(ChassisSpeeds speeds, boolean fieldOriented){
        swerveDrivePwr(speeds, fieldOriented, false);
    }

    public void swerveDrivePwr(ChassisSpeeds speeds, boolean fieldOriented, boolean isVelocity){
        boolean stopped = Math.abs(speeds.vxMetersPerSecond) < 0.002
                        && Math.abs(speeds.vyMetersPerSecond) < 0.002
                        && Math.abs(speeds.omegaRadiansPerSecond) < 0.002;

        if(!isVelocity) {
            limitSpeeds(speeds, k.fieldModePwr);
            //scale to m/s
            //TODO: make cleaner
            //speeds = speeds.times(k.maxWheelSpeed);
            speeds.vxMetersPerSecond *= k.maxWheelSpeed;
            speeds.vyMetersPerSecond *= k.maxWheelSpeed;
            double radius = k.wheelBL.wheelLocation.getNorm();
            speeds.omegaRadiansPerSecond *= k.maxWheelSpeed / radius / 1.2;
        } else {

        }
        
        if(fieldOriented){
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAngle());
            speeds = ChassisSpeeds.discretize(speeds, 0.02);
        }
        
        SwerveModuleState[] wheelStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(wheelStates, k.maxWheelSpeed);

        SwerveModuleState[]  optimizedWheelStates = new SwerveModuleState[4]; 

        for(int i = 0; i < 4; i++){
            optimizedWheelStates[i] = wheels[i].moveWheel(wheelStates[i], stopped, isVelocity);
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

        Logger.recordOutput("Drive/RobotRoll", inputs.pitch.getDegrees());

        //update odometry
        odometry.update(inputs.yaw, getWheelPositions());

        //update apriltags
        //coming soon to a robot near you

        robotPose = odometry.getEstimatedPosition();
        robotRelVelocity = kinematics.toChassisSpeeds(getWheelStates());

        double accelMag = inputs.accelVec.getNorm();
        Logger.recordOutput("Drive/Accel", accelMag);
    }

    @AutoLogOutput(key = "Drive/RobotPose")
    public Pose2d getPose(){
        return robotPose;
    }

    @AutoLogOutput(key = "Drive/RobotAngle")
    public Rotation2d getAngle(){
        return robotPose.getRotation();
    }

    @AutoLogOutput(key = "Drive/RobotRelVelocity")
    public ChassisSpeeds getRelVelocity(){
        return robotRelVelocity;
    }

    public void resetFieldOrientedAngle(){
        //reset to 0
        resetFieldOrientedAngle(new Rotation2d());
    }

    public void resetFieldOrientedAngle(Rotation2d newAngle){
        resetFieldOdometry(new Pose2d(robotPose.getTranslation(), newAngle));
    }

    public void resetFieldOdometry(){
        //default to somewhere that is visible on the 2d field
        resetFieldOdometry(new Pose2d(5, 5, getAngle()));
    }

    public void resetFieldOdometry(Pose2d newPose){
        odometry.resetPosition(inputs.yaw, getWheelPositions(), newPose);
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

    //note this modifies the object in place
    public void limitSpeeds(ChassisSpeeds speeds, double limit){
        if(Math.abs(speeds.vxMetersPerSecond) > limit) {
            speeds.vxMetersPerSecond = limit * Math.signum(speeds.vxMetersPerSecond);
        }
        if(Math.abs(speeds.vyMetersPerSecond) > limit) {
            speeds.vyMetersPerSecond = limit * Math.signum(speeds.vyMetersPerSecond);
        }
        if(Math.abs(speeds.omegaRadiansPerSecond) > limit) {
            speeds.omegaRadiansPerSecond = limit * Math.signum(speeds.omegaRadiansPerSecond);
        }
    }

    public void setBrake(boolean on){
        for(Wheel w : wheels){
            w.setBrake(on);
        }
    }
}