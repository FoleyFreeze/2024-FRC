package frc.robot.subsystems.motor;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class SparkMotor implements Motor{
    
    MotorCal k;
    CANSparkMax motor;

    RelativeEncoder encoder;
    SparkPIDController PIDController;

    int errorCount;

    public SparkMotor(MotorCal k){
        this.k = k;
        motor = new CANSparkMax(k.channel, MotorType.kBrushless);

        motor.restoreFactoryDefaults();

        encoder = motor.getEncoder();
        PIDController = motor.getPIDController();

        motor.setInverted(k.inverted);

        if(k.rampRate != 0){
            motor.setOpenLoopRampRate(k.rampRate);
            motor.setClosedLoopRampRate(k.rampRate);
        }

        PIDController.setP(k.p);
        PIDController.setI(k.i);
        PIDController.setIZone(k.iZone);
        PIDController.setD(k.d);
        PIDController.setFF(k.ff);
        PIDController.setDFilter(k.dFilt);

        powerLimMax = k.pidLimUp;
        powerLimMin = k.pidLimDn;
        PIDController.setOutputRange(k.pidLimDn,k.pidLimUp);

        if(k.currLim != 0){
            motor.setSmartCurrentLimit((int) k.currLim);
            motor.setSecondaryCurrentLimit(k.currLim2);
        }

        if(k.brakeMode){
            motor.setIdleMode(IdleMode.kBrake);
            brakeMode = true;
        } else {
            motor.setIdleMode(IdleMode.kCoast);
            brakeMode = false;
        }

        errorCount = 0;
    }

    @Override
    public void setPower(double power) {
        motor.set(power);
    }

    @Override
    public double getPosition() {
        double value = encoder.getPosition() * k.gearRatio;//in rotations
        REVLibError error = motor.getLastError();
        if(error != REVLibError.kOk){
            Logger.recordOutput("Motor"+k.channel, error.toString());
            errorCount++;
        }

        return value;
    }

    @Override
    public Rotation2d getRotation() {
        return new Rotation2d(Units.rotationsToRadians(getPosition()));
    }

    @Override
    public void setPosition(double position){
        PIDController.setReference(position / k.gearRatio, ControlType.kPosition);//in rotations
    }

    @Override
    public void setRotation(Rotation2d angle){
        double targetAngle = angle.getRotations();
        double rotations = getPosition();
        double wholeRotations = Math.floor(rotations);
        double adjustedReferenceAngle = targetAngle + wholeRotations;
        double fractionalAngle = rotations - wholeRotations;

        if(targetAngle - fractionalAngle > 0.5){
            adjustedReferenceAngle--;
        }else if (targetAngle - fractionalAngle < -0.5) {
            adjustedReferenceAngle++;
        }
        setPosition(adjustedReferenceAngle);
    }

    @Override
    public void setEncoderPosition(double position) {
        REVLibError err = encoder.setPosition(position / k.gearRatio);
        if(!err.equals(REVLibError.kOk)){
            System.out.println("Error resetting motor " + k.channel + ": " + err.toString());
        }
    }

    @Override
    public double getCurrent(){
        double value = motor.getOutputCurrent();
        REVLibError error = motor.getLastError();
        if(error != REVLibError.kOk){
            Logger.recordOutput("Motor"+k.channel, error.toString());
            errorCount++;
        }

        return value;
    }

    @Override
    public double getTemp(){
        double value = motor.getMotorTemperature();
        REVLibError error = motor.getLastError();
        if(error != REVLibError.kOk){
            Logger.recordOutput("Motor"+k.channel, error.toString());
            errorCount++;
        }

        return value;
    }

    boolean brakeMode;
    @Override
    public void setBrakeMode(boolean brakeMode) {
        if(brakeMode != this.brakeMode){
            this.brakeMode = brakeMode;
            if(brakeMode){
                motor.setIdleMode(IdleMode.kBrake);
            } else {
                motor.setIdleMode(IdleMode.kCoast);
            }
        }
    }

    double powerLimMin,powerLimMax;
    @Override
    public void setPIDPwrLim(double pwrLim) {
        if(pwrLim != powerLimMax){
            PIDController.setOutputRange(-pwrLim,pwrLim);
            powerLimMax = pwrLim;
            powerLimMin = -pwrLim;
        }
    }

    @Override
    public void setPIDPwrLim(double pwrLimPos, double pwrLimNeg) {
        if(pwrLimPos != powerLimMax || pwrLimNeg != powerLimMin){
            PIDController.setOutputRange(pwrLimNeg,pwrLimPos);
            powerLimMax = pwrLimPos;
            powerLimMin = pwrLimNeg;
        }
    }

    
    @Override
    public double getVelocity(){//in meters / sec
        double value = encoder.getVelocity() * k.gearRatio / 60.0;
        REVLibError error = motor.getLastError();
        if(error != REVLibError.kOk){
            Logger.recordOutput("Motor"+k.channel, error.toString());
            errorCount++;
        }

        return value;
    }

     @Override
    public void setVelocity(double velocity){
        PIDController.setReference(velocity / k.gearRatio * 60, ControlType.kVelocity);
    }

    @Override
    public double getVoltage(){
        double value = motor.getAppliedOutput() * motor.getBusVoltage();
    REVLibError error = motor.getLastError();
        if(error != REVLibError.kOk){
            Logger.recordOutput("Motor"+k.channel, error.toString());
            errorCount++;
        }

        return value;
    }

    @Override
    public void setVoltage(double voltage){
        motor.setVoltage(voltage);
    }

    @Override
    public int getErrorCount(){
        return errorCount;
    }
}
