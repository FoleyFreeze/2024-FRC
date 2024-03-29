package frc.robot.cals;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.motor.MotorCal;
import frc.robot.subsystems.motor.MotorCal.TypeMotor;

public class ShooterCals {
    public boolean disable = false;

    public double initAngleJog = 0;
    public double initSpeedJog = 0;
    public double jogAngleIncriment = .5;
    public double jogSpeedIncriment = 200;

    //note these are in inches (get converted later) //TODO: add a spot at 187 (4th shoot pos)
    double di = 36;             //in front, podium, 19ft
    public double camDistance[] = {di+17,  102,  113, di+14*12,  di+19*12};
    public double camAngle[] =    {55,     42,  40.5,   24.75,    24};
    public double camRPM[] =      {5000,   6000, 6000, 6500,     6630};
    public double camVelocity[] = {0, 0};
    
 
    public double maxFixedAngle = 61;
    public double minFixedAngle = 22;
    public double maxFixedSpeed = 10000;
    public double minFixedSpeed = 500;
    public double fixedAngle[] = {57, 35, 35}; // was 36.7, 36.7};//note far shot is cal'd out in inputs
    public double fixedRPM[] = {5500, 6000, 6000};

    public double ctrlBoardShootAngle = 59;
    public double ctrlBoardShootSpeed = 3000;
 
    public String fixedName[] = {"layup", "freethrow", "threepoint"};

    public double startAngle = 55;
    public double homePosition = 35;

    public MotorCal angleMotor = new MotorCal(TypeMotor.SPARK, 16)
                                        .setBrakeMode(true)
                                        .setCurrLim(30)
                                        .setPIDF(1.5, 0, 0, 0)//TODO: go back to 2
                                        .setPIDPwrLim(0.3, -0.3)//TODO: go back to 0.4
                                        .setRatio(85 / 32.0 * 30/24.0); //32 rotations of the screw, but gear ratio of 30/24
                                                                        //so a real ratio of 95deg / 25.6 (yes its geared up)

    public MotorCal shootMotorTop = new MotorCal(TypeMotor.FALCON, 13)
                                        .setPIDF(0.5, 0, 0, 0.1177)
                                        .setBrakeMode(false)
                                        .invert()
                                        .setCurrLim(60)
                                        .setPIDPwrLim(1, 0)
                                        .setRampRate(0.05)
                                        .setRatio(42 / 24.0); //geared up 42:24

    public MotorCal shootMotorBottom = new MotorCal(TypeMotor.FALCON, 14)
                                        .setPIDF(0.5, 0, 0, 0.1177)
                                        .setBrakeMode(false)
                                        .invert()
                                        .setCurrLim(60)
                                        .setPIDPwrLim(1, 0)
                                        .setRampRate(0.05)
                                        .setRatio(42 / 24.0); //geared up 42:24;

    public double allowedAngleError = 1;
    public double allowedRPMError = 100;

    public ShooterCals(){
        for(int i=0;i<camDistance.length;i++){
            camDistance[i] = Units.inchesToMeters(camDistance[i]);
        }
    }
}    