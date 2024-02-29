package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.auton.Locations;
import frc.robot.cals.ShooterCals;

public class Shooter extends SubsystemBase {
    public ShooterCals k;
    RobotContainer r;

    ShooterIO io;
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    double angleSetpoint;
    double rpmSetpoint;

    double speedJog;
    double angleJog;

    public Shooter (RobotContainer r, ShooterCals k){

        this.k = k;
        this.r = r;

        if(Robot.isReal() && !k.disable){
            io = new ShooterIO_HW(k);
        }else{
            io = new ShooterIO(){};
        }

        speedJog = k.initSpeedJog;
        angleJog = k.initAngleJog;

        io.setAngleEncoder(k.startAngle);
        angleSetpoint = k.startAngle;
        rpmSetpoint = 0;
    }


    public void jogSpeed(double delta){
        speedJog += delta;
    }

    public void jogAngle(double delta){
        angleJog += delta;
    }

    public void setAngle(double angle){
        io.setAngle(angle);
        angleSetpoint = angle;
    }

    public void fixedPrime(){
        int index = r.inputs.getFixedTarget();
        setAngle(k.fixedAngle[index] + angleJog);
        setRPM(k.fixedRPM[index] + speedJog);
    }

    public void visionPrime(){
        double distToTarget = Locations.tagSpeaker.minus(r.drive.getPose().getTranslation()).getNorm();
        double angle = interp(distToTarget, k.camDistance, k.camAngle);
        double rpm = interp(distToTarget, k.camDistance, k.camRPM);
        setAngle(angle + angleJog);
        setRPM(rpm + speedJog);
    }

    //TODO: is this still used?
    /*
    public void unShoot (){
        setAngle(k.homePosition);
        setRPM(0);
    }*/

    public void setRPM(double rpm){
        io.setShooterRPM(rpm);
        rpmSetpoint = rpm;
    }

    public void setShootPower(double power){
        io.setShooterVoltage(power*12);
    }

    public boolean checkAngleError(){
        return Math.abs(inputs.anglePosition - angleSetpoint) < k.allowedAngleError;
    }

    public boolean checkRPMError(){
        double avg = (inputs.shootBottomVelocity + inputs.shootTopVelocity)/2.0;
        return Math.abs(rpmSetpoint - avg) < k.allowedRPMError;
    }

    public void goHome(){
        setRPM(0);
        setAngle(k.homePosition);
    }

    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("shooter", inputs);

        SmartDashboard.putNumber("shoot angle", inputs.anglePosition);
        SmartDashboard.putNumber("shoot speeds bottom", inputs.shootBottomVelocity);
        SmartDashboard.putNumber("shoot speeds top", inputs.shootTopVelocity);
    }

    public double interp(double value, double axis[], double table[]){
        if (value < axis[0]){
            return table[0];
        }else if(value > axis[axis.length - 1]){
            return table[table.length - 1];
        }

        for (int i = 1; i < axis.length; i++){
            if (value < axis[i]){
                double t = MathUtil.inverseInterpolate(axis[i - 1], axis[i], value);
                return MathUtil.interpolate(table[i - 1], table[i], t);
            }
        }
        return table[table.length - 1];
    }
}
