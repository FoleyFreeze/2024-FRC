package frc.robot.cals;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.cals.DriveCals.WheelCal;
import frc.robot.subsystems.motor.MotorCal;
import frc.robot.subsystems.motor.MotorCal.TypeMotor;

public class DriveCals {

    public class WheelCal{
        public MotorCal driveMotor;
        public MotorCal swerveMotor;

        public Translation2d wheelLocation;
        
    }

    //wheel locations
    private double width = Units.inchesToMeters(12); //robo dimensions 
    private double length = Units.inchesToMeters(13);


    public double maxWheelSpeed = Units.feetToMeters(18);
    
    //not the actual numbers
    double swerveKp = 0.4;
    double swerveKi = 0.000001;
    double swerveIZone = 0.0;
    double swerveKd = 1.3;
    double swerveDfilt = 0.0;
    double swerveKf = 0;
    double swerveLim = 0.7;
    double swerveCurrLim = 45;

    public WheelCal wheelFL = new WheelCal();{
        wheelFL.driveMotor = new MotorCal(TypeMotor.SPARK, 6);
        wheelFL.swerveMotor = new MotorCal(TypeMotor.SPARK, 7).invert().setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setIZone(swerveIZone).setPIDPwrLim(swerveLim).setCurrLim(swerveCurrLim);
        wheelFL.wheelLocation = new Translation2d(-width, length);
    }

    public WheelCal wheelFR = new WheelCal();{
        wheelFR.driveMotor = new MotorCal(TypeMotor.SPARK, 2);
        wheelFR.swerveMotor = new MotorCal(TypeMotor.SPARK, 3).invert().setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setIZone(swerveIZone).setPIDPwrLim(swerveLim).setCurrLim(swerveCurrLim);
        wheelFR.wheelLocation = new Translation2d(width, length);
    }

    public WheelCal wheelBL = new WheelCal();{
        wheelBL.driveMotor = new MotorCal(TypeMotor.SPARK, 8);
        wheelBL.swerveMotor = new MotorCal(TypeMotor.SPARK, 9).invert().setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setIZone(swerveIZone).setPIDPwrLim(swerveLim).setCurrLim(swerveCurrLim);
        wheelBL.wheelLocation = new Translation2d(-width, -length);
    }

    public WheelCal wheelBR = new WheelCal();{
        wheelBR.driveMotor = new MotorCal(TypeMotor.SPARK, 20);
        wheelBR.swerveMotor = new MotorCal(TypeMotor.SPARK, 1).invert().setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setIZone(swerveIZone).setPIDPwrLim(swerveLim).setCurrLim(swerveCurrLim);
        wheelBR.wheelLocation = new Translation2d(width, -length);
    }

    public WheelCal[] wheelCals = {wheelFL, wheelFR, wheelBL, wheelBR};

    public double fieldModePwr = 0.95;
    public double pitModePwr = 0.3;


    
}
