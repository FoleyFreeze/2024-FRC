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
    public ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    double angleSetpoint;
    public double rpmSetpoint;

    public double speedJog;
    public double speedJogLob;
    public double angleJog;

    public Shooter (RobotContainer r, ShooterCals k){

        this.k = k;
        this.r = r;

        if(Robot.isReal() && !k.disable){
            io = new ShooterIO_HW(k);
        }else{
            io = new ShooterIO(){};
        }

        speedJog = k.initSpeedJog;
        speedJogLob = 0;
        angleJog = k.initAngleJog;

        //TODO: see if this works here
        io.updateInputs(inputs);
        Logger.processInputs("shooter", inputs);

        if(Math.abs(inputs.anglePosition) < 20){ 
            io.setAngleEncoder(k.startAngle);
            System.out.println("Successfully reset shoot angle position from: " + inputs.anglePosition);
             
        } else {
            System.out.println("Did not need to reset shooter angle. Already at: " + inputs.anglePosition);
        }
        angleSetpoint = k.startAngle;
        rpmSetpoint = 0;
    }

    public void resetAngle(){
        io.setAngleEncoder(k.startAngle);
        System.out.println("Zero'd Shooter");
    }


    public void jogSpeed(double delta){
        speedJog += delta;
        System.out.println("Shoot Speed Jog is: " + speedJog);
    }

    public void jogLobSpeed(double delta){
        speedJogLob += delta;
        System.out.println("Shoot Lob Speed Jog is: " + speedJogLob);
    }

    public void jogAngle(double delta){
        angleJog += delta;
        System.out.println("Shoot Angle Jog is: " + angleJog);
    }

    public void setAngle(double angle){
        io.setAngle(angle);
        angleSetpoint = angle;
    }

    public double getTestAngle(){
        double angleIdx = r.inputs.getLeftDial();
        double output = angleIdx * (k.maxFixedAngle - k.minFixedAngle) + k.minFixedAngle;
        return output;
    }

    public double getTestSpeed(){
        double speedIdx = r.inputs.getRightDial();
        double output = speedIdx * (k.maxFixedSpeed - k.minFixedSpeed) + k.minFixedSpeed;
        return output;
    }
    public void fixedPrime(){
        //TODO: go back to this when done testing
        int index = r.inputs.getFixedTarget();
        
        //jog lob shots independently
        double jog = speedJog;
        if(index == 1) jog = speedJogLob;

        setAngle(k.fixedAngle[index] + angleJog); //COMMENT this out to tune shooter with dials on Flysky
        setRPM(k.fixedRPM[index] + jog);     //COMMENT this out to tune shooter with dials on Flysky

        //setAngle(getTestAngle()); //UNCOMMENT this for tuning shooter with dials on FlySky
        //setRPM(getTestSpeed());   //UNCOMMENT this for tuning shooter with dials on FlySky
    }

    public void commandPrime(double angle, double rpm){
        setAngle(angle + angleJog);
        setRPM(rpm + speedJog);
    }

    public void visionPrime(){
        double distToTarget = Locations.tagSpeaker.minus(r.drive.getPose().getTranslation()).getNorm();
        distancePrime(distToTarget);
    }

    public void visionLob(){
        double dist = Locations.tagSpeaker.minus(r.drive.getPose().getTranslation()).getNorm();
        double angle = interp(dist, k.camLobDist, k.camLobAngle);
        double rpm = interp(dist, k.camLobDist, k.camLobRPM);
        setAngle(angle + angleJog);
        setRPM(rpm + speedJog);
    }

    public double getLobOffset(double dist){
        return Math.toRadians(interp(dist, k.camLobDist, k.camLobBotAngleOffset));
    }

    public double getBotAngleTol(double dist){
        return Math.toRadians(interp(dist, k.camDistTol, k.camAngleTol));
    }

    public void distancePrime(double distance){
        double angle = interp(distance, k.camDistance, k.camAngle);
        double rpm = interp(distance, k.camDistance, k.camRPM);
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
        rpmSetpoint = 0;
    }

    public boolean checkAngleError(){
        return Math.abs(inputs.anglePosition - angleSetpoint) < k.allowedAngleError;
    }

    public boolean checkRPMError(){
        //if zero we are not up to speed
        if(rpmSetpoint == 0) return false;

        double avg = (inputs.shootBottomVelocity + inputs.shootTopVelocity)/2.0;
        return Math.abs(rpmSetpoint - avg) < k.allowedRPMError;
    }

    public void goHome(){
        setShootPower(0);
        setAngle(k.homePosition);
    }

    public double getShooterCurrent(){
        return (inputs.shootBottomCurrentAmps + inputs.shootTopCurrentAmps) / 2.0;
    }

    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter/AngleSetpoint", angleSetpoint);
        Logger.recordOutput("Shooter/RPMSetpoint", rpmSetpoint);

        SmartDashboard.putNumber("shoot angle", inputs.anglePosition);
        SmartDashboard.putNumber("shoot speeds bottom", inputs.shootBottomVelocity);
        SmartDashboard.putNumber("shoot speeds top", inputs.shootTopVelocity);

        SmartDashboard.putNumber("Test Angle", getTestAngle());
        SmartDashboard.putNumber("Test Speed", getTestSpeed());
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
