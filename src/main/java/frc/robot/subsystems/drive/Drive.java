package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.cals.DriveCals;
import frc.robot.subsystems.motor.Motor;
import frc.robot.util.FileManager;

public class Drive extends SubsystemBase{
    
    RobotContainer r;
    public DriveCals k;
    SwerveDriveKinematics kinematics;
    WheelModule[] wheels;
    AHRS navX;
    SwerveDriveOdometry odometry;

    FileManager fm = new FileManager("/home/lvuser/WheelEncoderOffsets.txt");

    class WheelModule{
        Motor driveMotor;
        Motor swerveMotor;
        AnalogInput absEncoder;
        double encAngOffset;

        public SwerveModulePosition getPosition(){
            Rotation2d angle = swerveMotor.getRotation();
            double driveEnc = Units.inchesToMeters(driveMotor.getPosition());
            
            return new SwerveModulePosition(driveEnc, angle);
        }
    }

    public Drive (RobotContainer r, DriveCals k){
        this.r = r;
        this.k = k;

        navX = new AHRS(Port.kMXP);

        wheels = new WheelModule[4];
    
        for (int i = 0; i<4; i++){
            wheels[i] = new WheelModule();
            wheels[i].swerveMotor = Motor.create(k.wheelCals[i].swerveMotor);
            wheels[i].driveMotor = Motor.create(k.wheelCals[i].driveMotor);
            wheels[i].absEncoder = new AnalogInput(k.wheelCals[i].encChannel);
        }

        kinematics = new SwerveDriveKinematics(
            k.wheelCals[0].wheelLocation,
            k.wheelCals[1].wheelLocation,
            k.wheelCals[2].wheelLocation,
            k.wheelCals[3].wheelLocation
            );

        odometry = new SwerveDriveOdometry(
            kinematics, 
            getRobotAngle(),
            new SwerveModulePosition[] {
                wheels[0].getPosition(),
                wheels[1].getPosition(),
                wheels[2].getPosition(),
                wheels[3].getPosition()
            }
        );

        


    }
 
    public Rotation2d getRobotAngle(){
        SmartDashboard.putNumber("Yaw", navX.getYaw());
        SmartDashboard.putNumber("Angle", navX.getAngle());
        SmartDashboard.putNumber("FusedHeading", navX.getFusedHeading());
        
        return new Rotation2d(Units.degreesToRadians(-navX.getYaw()));
    }
    
    public void swerveDrivePwr(ChassisSpeeds speeds, boolean fieldOriented){
        boolean stopped = Math.abs(speeds.vxMetersPerSecond) < 0.02
                        && Math.abs(speeds.vyMetersPerSecond) < 0.02
                        && Math.abs(speeds.omegaRadiansPerSecond) < 0.02;
        
        //scale to m/s
        speeds = speeds.times(k.maxWheelSpeed);
        
        if(fieldOriented){
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRobotAngle());
        }
        
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, k.maxWheelSpeed);

        for(int i = 0; i < 4; i++){
            Rotation2d currRot = wheels[i].swerveMotor.getRotation();
            SwerveModuleState finalState = SwerveModuleState.optimize(moduleStates[i], currRot);
            if(!stopped) wheels[i].swerveMotor.setRotation(finalState.angle.plus(Rotation2d.fromDegrees(wheels[i].encAngOffset)));
            wheels[i].driveMotor.setPower(finalState.speedMetersPerSecond / k.maxWheelSpeed);
        }

    }

    public void readAbsOffset(){
        try{
            if(fm.exists()){
                System.out.println("Reading wheel positions file:");
                for(int i=0;i<4;i++) {
                    double offset = Double.parseDouble(fm.readLine());
                    System.out.println(k.wheelCals[i].name + " " + offset);
                    setEncAngOffset(i, offset);
                }
                fm.close();
                }
        }catch(Exception e){
            System.out.println(e.toString());
            e.printStackTrace();
            //if there was an error, reset to cal value
            System.out.println("Error reading file, defaulting to:");
            for(int i = 0; i < 4; i++) {
                double v = wheels[i].absEncoder.getVoltage();
                setEncAngOffset(i, v);
                System.out.println(k.wheelCals[i].name + "  :)  " + v);
            }
        }
    }

    public void writeAbsOffset(){
        try{
            System.out.println("Saving new wheel locations:");
            for(int i = 0; i < 4; i++){
                double voltage = wheels[i].absEncoder.getVoltage();
                fm.writeLine(Double.toString(voltage));
                setEncAngOffset(i, voltage);
                System.out.println(k.wheelCals[i].name + "  ):  " + voltage);
            }
            fm.close();
        }catch(Exception e){
            System.out.println("Error while saving wheel locations:");
            System.out.println(e.toString());
            e.printStackTrace();
        }
    }

    public void setEncAngOffset(int i, double offset){
        wheels[i].swerveMotor.setEncoderPosition(0.0);
        //90 deg offset not needed if zeroing with wheels pointed forward (bevels to the right, -y direction)
        wheels[i].encAngOffset = (wheels[i].absEncoder.getVoltage() - offset) / 5.0 * 2 * Math.PI /*- Math.PI / 2.0*/;
        System.out.println(k.wheelCals[i].name + " angle offset: " + Units.radiansToDegrees(wheels[i].encAngOffset));
        System.out.println(k.wheelCals[i].name + "current voltage: " + wheels[i].absEncoder.getVoltage());
    }

    Pose2d robotPose;

    @Override
    public void periodic(){
        //update odometry
        robotPose = odometry.update(
            getRobotAngle(), 
            new SwerveModulePosition[] {
                wheels[0].getPosition(),
                wheels[1].getPosition(),
                wheels[2].getPosition(),
                wheels[3].getPosition()
            }
        );

        
    }

    public Pose2d getPose(){
        return robotPose;
    }
}
