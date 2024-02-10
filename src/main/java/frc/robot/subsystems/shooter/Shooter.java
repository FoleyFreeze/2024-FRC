package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.cals.ShooterCals;

public class Shooter extends SubsystemBase {
    ShooterCals k;
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
    }

    //TODO: jog up/down functions

    public void setAngle(double angle){
        io.setAngle(angle);
        angleSetpoint = angle;
    }

    public void fixedPrime (){
        int index = r.inputs.getFixedTarget();
        setAngle(k.fixedAngle[index] + angleJog);
        setRPM(k.fixedRPM[index] + speedJog);
    }

    public void unShoot (){
        setAngle(28);
        setRPM(0);
    }

    public void setRPM(double rpm){
        io.setShooterRPM(rpm);
        rpmSetpoint = rpm;
    }

    public boolean checkAngleError(){
        return Math.abs(inputs.anglePosition - angleSetpoint) < k.allowedAngleError;
    }

    public boolean checkRPMError(){
        double avg = (inputs.shootBottomVelocity + inputs.shootTopVelocity)/2;
        return Math.abs(rpmSetpoint - avg) < k.allowedRPMError;
    }

    public void goHome(){
        setRPM(0);
        setAngle(k.homePosition);
    }

    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("shooter", inputs);
    }
}
