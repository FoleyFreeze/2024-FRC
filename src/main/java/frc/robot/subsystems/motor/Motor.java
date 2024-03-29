package frc.robot.subsystems.motor;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Motor {
    /* The motor class is designed to abstract away specific
     * motor conversion factors or functions that slow the
     * process of creating a subsystem. Instead of specific
     * sparks or talons being called in the code, an instance
     * of this class is used. It also allows the type of motor
     * to be changed universally and easily through the motor
     * type variable.
     */

     public static Motor create(MotorCal cal){
        switch(cal.type){
            case SPARK:
                return new SparkMotor(cal);
            case FALCON:
                return new TalonFXMotor(cal);
            default:
                return new NullMotor(cal);
        }
    }

    //Functions required for every motor to utiliize
    public abstract void setPower(double power);
    public abstract double getPosition();
    public abstract void setPosition(double position);
    public abstract void setEncoderPosition(double position);
    public abstract double getCurrent();
    public abstract double getTemp();
    public abstract void setBrakeMode(boolean brakeMode);
    public abstract void setPIDPwrLim(double pwrLim);
    public abstract void setPIDPwrLim(double posLim, double negLim);
    public abstract Rotation2d getRotation();
    public abstract void setRotation(Rotation2d angle);
    public abstract void setVelocity(double velocity);
    public default double getVelocity(){return 0;};
    public default double getVoltage(){return 0;};
    public default void setVoltage(double voltage){};
    public default int getErrorCount(){return 0;};
}
