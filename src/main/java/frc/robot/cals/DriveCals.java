package frc.robot.cals;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.motor.MotorCal;
import frc.robot.subsystems.motor.MotorCal.TypeMotor;

public class DriveCals {

    public boolean disable = true;

    public class WheelCal{
        public boolean disable = true;

        public MotorCal driveMotor;
        public MotorCal swerveMotor;
        public int encChannel;

        public Translation2d wheelLocation;
        
        public String name;

        public double maxSpeed = maxWheelSpeed;
    }

    //wheel locations
    private double width = Units.inchesToMeters(20.75 /2.0); //robo dimensions 
    private double length = Units.inchesToMeters(21 /2.0);

    public double maxWheelSpeed = Units.feetToMeters(16);
    
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
    double driveDiameter = Units.inchesToMeters(4);
    double conversionFactor = driveRatio * Math.PI * driveDiameter;

    public WheelCal wheelFL = new WheelCal();{
        wheelFL.driveMotor = new MotorCal(TypeMotor.SPARK, 6).setRatio(conversionFactor).invert();
        wheelFL.swerveMotor = new MotorCal(TypeMotor.SPARK, 7).invert().setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setIZone(swerveIZone).setPIDPwrLim(swerveLim).setCurrLim(swerveCurrLim).setRatio(swerveRatio);
        wheelFL.wheelLocation = new Translation2d(width, length);
        wheelFL.encChannel = 2;
        wheelFL.name = "FL";
    }

    public WheelCal wheelFR = new WheelCal();{
        wheelFR.driveMotor = new MotorCal(TypeMotor.SPARK, 2).setRatio(conversionFactor);
        wheelFR.swerveMotor = new MotorCal(TypeMotor.SPARK, 3).setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setIZone(swerveIZone).setPIDPwrLim(swerveLim).setCurrLim(swerveCurrLim).setRatio(swerveRatio);
        wheelFR.wheelLocation = new Translation2d(width, -length);
        wheelFR.encChannel = 3;
        wheelFR.name = "FR";
    }

    public WheelCal wheelBL = new WheelCal();{
        wheelBL.driveMotor = new MotorCal(TypeMotor.SPARK, 8).setRatio(conversionFactor);
        wheelBL.swerveMotor = new MotorCal(TypeMotor.SPARK, 9).invert().setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setIZone(swerveIZone).setPIDPwrLim(swerveLim).setCurrLim(swerveCurrLim).setRatio(swerveRatio);
        wheelBL.wheelLocation = new Translation2d(-width, length);
        wheelBL.encChannel = 1;
        wheelBL.name = "BL";
    }

    public WheelCal wheelBR = new WheelCal();{
        wheelBR.driveMotor = new MotorCal(TypeMotor.SPARK, 1).setRatio(conversionFactor);
        wheelBR.swerveMotor = new MotorCal(TypeMotor.SPARK, 20).invert().setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setIZone(swerveIZone).setPIDPwrLim(swerveLim).setCurrLim(swerveCurrLim).setRatio(swerveRatio);
        wheelBR.wheelLocation = new Translation2d(-width, -length);
        wheelBR.encChannel = 0;
        wheelBR.name = "BR";
    }

    public WheelCal[] wheelCals = {wheelFL, wheelFR, wheelBL, wheelBR};

    public double fieldModePwr = 0.95;
    public double pitModePwr = 0.3;


    
}
