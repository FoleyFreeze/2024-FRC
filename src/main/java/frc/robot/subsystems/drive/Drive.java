package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    public SwerveDrivePoseEstimator odometry;

    public Pose2d robotPose = new Pose2d();
    public ChassisSpeeds robotRelVelocity = new ChassisSpeeds();

    //used for determining if robot is aligned for camera shoot 
    public boolean atAngleSetpoint = false;

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
        //attempt to back calculate an accel feedfoward

        double dt = Timer.getFPGATimestamp() - prevTime;
        prevTime = Timer.getFPGATimestamp();

        //calculate what the velocity setpoint would have been without the position PID
        Translation2d poseError = targetPose.getTranslation().minus(currentPose.getTranslation());
        Translation2d extraVelAdded = poseError.times(profilePID.kP);
        ChassisSpeeds fieldRelSpeedsRaw = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getAngle());
        ChassisSpeeds fieldRelSpeedsFF = new ChassisSpeeds();
        fieldRelSpeedsFF.vxMetersPerSecond = fieldRelSpeedsRaw.vxMetersPerSecond - extraVelAdded.getX();
        fieldRelSpeedsFF.vyMetersPerSecond = fieldRelSpeedsRaw.vyMetersPerSecond - extraVelAdded.getY();

        //reset when a new path is generated
        Logger.recordOutput("Drive/Accels2/NewPath", newPath);
        if(newPath){
            prevFieldRelChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getRelVelocity(), getAngle());
            newPath = false;
        }
        ChassisSpeeds accel2 = fieldRelSpeedsFF.minus(prevFieldRelChassisSpeeds).div(dt);
        prevFieldRelChassisSpeeds = fieldRelSpeedsFF;
        ChassisSpeeds botRelAccel2 = ChassisSpeeds.fromFieldRelativeSpeeds(accel2, getAngle());
        Logger.recordOutput("Drive/Accels2/FieldRelSpeedsRaw", fieldRelSpeedsRaw);
        Logger.recordOutput("Drive/Accels2/FieldRelSpeedsFF", fieldRelSpeedsFF);
        Logger.recordOutput("Drive/Accels2/FieldRelAccel", accel2);
        Logger.recordOutput("Drive/Accels2/BotRelAccel", botRelAccel2);
        Logger.recordOutput("Drive/Accels2/dt", dt);

        //limit the calculated accel to the constraints placed on the path
        ChassisSpeeds limitedAccel2 = new ChassisSpeeds(botRelAccel2.vxMetersPerSecond, botRelAccel2.vyMetersPerSecond, botRelAccel2.omegaRadiansPerSecond);
        double maxAccel = Math.hypot(botRelAccel2.vxMetersPerSecond, botRelAccel2.vyMetersPerSecond);
        if(maxAccel > profileConstraints.getMaxAccelerationMpsSq()){
            double ratio = maxAccel / profileConstraints.getMaxAccelerationMpsSq();
            limitedAccel2.vxMetersPerSecond /= ratio;
            limitedAccel2.vyMetersPerSecond /= ratio;
        }
        Logger.recordOutput("Drive/Accels2/LimitedBotRelAccel", limitedAccel2);

        
        //use as velocity this time
        swerveDrivePwr(speeds, false, limitedAccel2);
    }

    //for accel calcs
    public double prevTime;
    public boolean newPath = false;
    public Pose2d currentPose = new Pose2d();
    public Pose2d targetPose = new Pose2d();
    public PIDConstants profilePID = new PIDConstants(0);
    public ChassisSpeeds prevFieldRelChassisSpeeds = new ChassisSpeeds();
    public PathConstraints profileConstraints = new PathConstraints(0, 0, 0, 0);

    public void swerveDrivePwr(ChassisSpeeds speeds, boolean fieldOriented){
        swerveDrivePwr(speeds, fieldOriented, null);
    }

    public void swerveDrivePwr(ChassisSpeeds speeds, boolean fieldOriented, ChassisSpeeds accel){
        boolean stopped = Math.abs(speeds.vxMetersPerSecond) < 0.002
                        && Math.abs(speeds.vyMetersPerSecond) < 0.002
                        && Math.abs(speeds.omegaRadiansPerSecond) < 0.002;

        if(accel == null) {
            limitSpeeds(speeds, k.fieldModePwr);
            //scale to m/s
            //TODO: make cleaner
            //speeds = speeds.times(k.maxWheelSpeed);
            speeds.vxMetersPerSecond *= k.maxWheelSpeed;
            speeds.vyMetersPerSecond *= k.maxWheelSpeed;
            double radius = k.wheelBL.wheelLocation.getNorm();
            speeds.omegaRadiansPerSecond *= k.maxWheelSpeed / radius / 1.2;
        } else {
            //add acceleration to the requested chassis speeds
            //but only take the portion of the accel vector that points in the same direction as the velocity vector
            double accelAngle = Math.atan2(accel.vyMetersPerSecond, accel.vxMetersPerSecond);
            double velAngle = Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond);
            double accelHyp = Math.hypot(accel.vxMetersPerSecond, accel.vyMetersPerSecond) * Math.cos(accelAngle - velAngle);
            double vxAccel = accelHyp * Math.cos(velAngle) * k.kA * k.maxWheelSpeed;
            double vyAccel = accelHyp * Math.sin(velAngle) * k.kA * k.maxWheelSpeed;

            speeds.vxMetersPerSecond += vxAccel;
            speeds.vyMetersPerSecond += vyAccel;

            Logger.recordOutput("Drive/Accels2/AccelAngle", accelAngle);
            Logger.recordOutput("Drive/Accels2/VelAngle", velAngle);
            Logger.recordOutput("Drive/Accels2/AccelMagnitude", accelHyp);
            Logger.recordOutput("Drive/Accels2/AccelExtraXVel", vxAccel);
            Logger.recordOutput("Drive/Accels2/AccelExtraYVel", vyAccel);
        }
        
        if(fieldOriented){
            Rotation2d angle = getAngle();
            if(DriverStation.getAlliance().get() == Alliance.Red){
                angle = angle.plus(Rotation2d.fromDegrees(180));
            }
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, angle);
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
        Rotation2d zeroAngle = new Rotation2d();
        if(DriverStation.getAlliance().isPresent()){
            if(DriverStation.getAlliance().get() == Alliance.Red){
                zeroAngle = Rotation2d.fromDegrees(180);
            }
        }
        resetFieldOrientedAngle(zeroAngle);
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

    public void setSwerveBrake(boolean on){
        for(Wheel w: wheels){
            w.setSwerveBrake(on);
        }
    }

    Matrix<N3,N1> visionMat = VecBuilder.fill(0.9, 0.9, 0.9);
    public void applyVisionDataToOdometry(Pose2d visionBotLocation, double timestamp, double estAccuracy){
        Matrix<N3,N1> mat = visionMat.times(estAccuracy);
        odometry.addVisionMeasurement(visionBotLocation, timestamp, mat);
    }
}