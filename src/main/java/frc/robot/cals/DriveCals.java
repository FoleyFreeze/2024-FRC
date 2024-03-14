package frc.robot.cals;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.auton.Locations;
import frc.robot.subsystems.motor.MotorCal;
import frc.robot.subsystems.motor.MotorCal.TypeMotor;

public class DriveCals {

    public boolean disable = false;

    public class WheelCal{
        public boolean disable = false;

        public MotorCal driveMotor;
        public MotorCal swerveMotor;
        public int encChannel;

        public Translation2d wheelLocation;
        
        public String name;

        public double maxSpeed = maxWheelSpeed;
        public double wheelRadius = driveDiameter / 2.0;

        public boolean driveWithVel = false;

        //due to offset bevel gear in the swerve module the drive wheel moves when the module rotates
        //account for this when returning the drive wheel position
        public double rotationToDriveRatio = 15 / 45.0 * 28 / 16.0;
    }

    //wheel locations
    private double bumperWidth = Locations.robotWidth;
    private double bumperLength = Locations.robotLength;
    private double wheelWidth = Units.inchesToMeters(20.75);
    private double wheelLength = Units.inchesToMeters(23.25);
    //calculate correct center offset
    //this assumes that the side delta to the bumper is the 
    //  same as the back distance to the bumper and
    //  all of the remaining length difference is in front
    private double centerOffsetX = (bumperLength - wheelLength - (bumperWidth - wheelWidth)) /2.0;
    private double centerOffsetY = 0;
    private double wheelLocationX = wheelLength / 2.0;
    private double wheelLocationY = wheelWidth / 2.0;

    public double maxWheelSpeed = Units.feetToMeters(14);
    
    //TODO: current limits

    //not the actual numbers
    double swerveKp = 0.4;//0.4;
    double swerveKi = 0;//0.000001;
    double swerveIZone = 0.0;
    double swerveKd = 0.1;//1.3;
    double swerveDfilt = 0.0;
    double swerveKf = 0;
    double swerveLim = 0.5;
    double swerveCurrLim = 20;//45;
    double swerveRatio = 7 / 150.0;
    
    double driveRatio = 14 / 50.0 * 28 / 16.0 * 15 / 45.0; //mk4i L3
    double driveCurrLim = 60;
    double driveDiameter = Units.inchesToMeters(4);
    double conversionFactor = driveRatio * Math.PI * driveDiameter;

    public double fieldModePwr = 0.95;
    public double pitModePwr = 0.3;
    public double autoDrivePower = 0.9;

    public double autoGatherPower = 0.1;


    public WheelCal wheelFL = new WheelCal();{
        wheelFL.driveMotor = new MotorCal(TypeMotor.SPARK, 10)
            .setRatio(conversionFactor)
            .invert()
            .setBrakeMode(false)
            .setCurrLim(driveCurrLim);
        wheelFL.swerveMotor = new MotorCal(TypeMotor.SPARK, 11)
            .invert()
            .setPIDF(swerveKp, swerveKi, swerveKd, swerveKf)
            .setIZone(swerveIZone)
            .setPIDPwrLim(swerveLim)
            .setCurrLim(swerveCurrLim)
            .setRatio(swerveRatio)
            .setBrakeMode(true);
        wheelFL.wheelLocation = new Translation2d(wheelLocationX - centerOffsetX, wheelLocationY - centerOffsetY);
        wheelFL.encChannel = 2;
        wheelFL.name = "FL";
    }

    public WheelCal wheelFR = new WheelCal();{
        wheelFR.driveMotor = new MotorCal(TypeMotor.SPARK, 8)
            .setRatio(conversionFactor)
            .setCurrLim(driveCurrLim)
            .setBrakeMode(false);
        wheelFR.swerveMotor = new MotorCal(TypeMotor.SPARK, 9)
            //.invert()
            .setPIDF(swerveKp, swerveKi, swerveKd, swerveKf)
            .setIZone(swerveIZone)
            .setPIDPwrLim(swerveLim)
            .setCurrLim(swerveCurrLim)
            .setRatio(swerveRatio)
            .setBrakeMode(true);
        wheelFR.wheelLocation = new Translation2d(wheelLocationX - centerOffsetX, -wheelLocationY - centerOffsetY);
        wheelFR.encChannel = 3;
        wheelFR.name = "FR";
    }

    public WheelCal wheelBL = new WheelCal();{
        wheelBL.driveMotor = new MotorCal(TypeMotor.SPARK, 18)
            .setRatio(conversionFactor)
            .setCurrLim(driveCurrLim)
            .setBrakeMode(false);
        wheelBL.swerveMotor = new MotorCal(TypeMotor.SPARK, 19)  
            .invert()
            .setPIDF(swerveKp, swerveKi, swerveKd, swerveKf)
            .setIZone(swerveIZone)
            .setPIDPwrLim(swerveLim)
            .setCurrLim(swerveCurrLim)
            .setRatio(swerveRatio)
            .setBrakeMode(true);
        wheelBL.wheelLocation = new Translation2d(-wheelLocationX - centerOffsetX, wheelLocationY - centerOffsetY);
        wheelBL.encChannel = 1;
        wheelBL.name = "BL";
    }

    public WheelCal wheelBR = new WheelCal();{
        wheelBR.driveMotor = new MotorCal(TypeMotor.SPARK, 20)
            .setRatio(conversionFactor)
            .setCurrLim(driveCurrLim)
            .setBrakeMode(false);
        wheelBR.swerveMotor = new MotorCal(TypeMotor.SPARK, 1)
            .invert()
            .setPIDF(swerveKp, swerveKi, swerveKd, swerveKf)
            .setIZone(swerveIZone)
            .setPIDPwrLim(swerveLim)
            .setCurrLim(swerveCurrLim)
            .setRatio(swerveRatio)
            .setBrakeMode(true);
        wheelBR.wheelLocation = new Translation2d(-wheelLocationX - centerOffsetX, -wheelLocationY - centerOffsetY);
        wheelBR.encChannel = 0;
        wheelBR.name = "BR";
    }

    public WheelCal[] wheelCals = {wheelFL, wheelFR, wheelBL, wheelBR};

    public HolonomicPathFollowerConfig notePathFollowerConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(5, 0, 0),
        new PIDConstants(5, 0, 0),
        maxWheelSpeed,
        wheelFL.wheelLocation.getNorm(),
        new ReplanningConfig()
    );

    public HolonomicPathFollowerConfig AutonPathFollowerConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(5, 0, 0),
        new PIDConstants(4, 0, 0),
        maxWheelSpeed,
        wheelFL.wheelLocation.getNorm(),
        new ReplanningConfig()
    );

    public boolean dontFlip = false;
    public boolean flipPath(){ 
        if(dontFlip) return false;

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()){
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false; 
    }
}
