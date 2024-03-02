package frc.robot.subsystems.motor;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;

public class TalonFXMotor implements Motor {

    MotorCal k;
    TalonFX motor;

    TalonFXConfiguration cfg;

    VoltageOut targetVoltage;
    PositionVoltage targetPosition;
    VelocityVoltage targetVelocity;

    public TalonFXMotor (MotorCal k){
        this.k = k;

        motor = new TalonFX(k.channel);

        cfg = new TalonFXConfiguration();

        Slot0Configs pidConfig = new Slot0Configs();
        pidConfig.kP = k.p;
        pidConfig.kI = k.i;
        pidConfig.kD = k.d;
        pidConfig.kV = k.ff;
        cfg.Slot0 = pidConfig;

        if(k.inverted){
            cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        } else {
            cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        if(k.brakeMode){
            cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }

        cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = k.rampRate;
        cfg.OpenLoopRamps.VoltageOpenLoopRampPeriod = k.rampRate;

        if(k.currLim != 0){
            cfg.CurrentLimits.StatorCurrentLimit = k.currLim;
            cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        }

        //convert to volts
        cfg.Voltage.PeakForwardVoltage = k.pidLimUp * 12;
        cfg.Voltage.PeakReverseVoltage = k.pidLimDn * 12;

        motor.getConfigurator().apply(cfg);

        targetPosition = new PositionVoltage(0);
        targetVelocity = new VelocityVoltage(0);
        targetVoltage = new VoltageOut(0);
    }

    @Override
    public void setPower(double power) {
        targetVoltage.Output = power * 12;
        motor.setControl(targetVoltage);
    }

    @Override
    public double getPosition() {
        return motor.getPosition().getValueAsDouble() * k.gearRatio; //in rotations
    }

    @Override
    public void setPosition(double position) {
        targetPosition.Position = position / k.gearRatio;
        motor.setControl(targetPosition);
    }

    @Override
    public void setEncoderPosition(double position) {
        motor.setPosition(position / k.gearRatio);
    }

    @Override
    public double getCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public double getTemp() {
        return motor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setBrakeMode(boolean brakeMode) {
        throw new UnsupportedOperationException("Unimplemented method 'setBrakeMode'");
    }

    @Override
    public void setPIDPwrLim(double pwrLim) {
        setPIDPwrLim(pwrLim, -pwrLim);
    }

    @Override
    public void setPIDPwrLim(double pwrLimPos, double pwrLimNeg){
        cfg.Voltage.PeakForwardVoltage = pwrLimPos*12;
        cfg.Voltage.PeakReverseVoltage = pwrLimNeg*12;
        motor.getConfigurator().apply(cfg.Voltage);
    }

    @Override
    public Rotation2d getRotation() {
        throw new UnsupportedOperationException("Unimplemented method 'getRotation'");
    }

    @Override
    public void setRotation(Rotation2d angle) {
        throw new UnsupportedOperationException("Unimplemented method 'setRotation'");
    }

    @Override
    public void setVelocity(double velocity) {
        targetVelocity.Velocity = velocity / k.gearRatio / 60.0;//convert to revs per second
        motor.setControl(targetVelocity);
    }

    @Override
    public double getVelocity(){
        return motor.getVelocity().getValueAsDouble() * k.gearRatio * 60.0;//convert to rpm
    }
    
}
