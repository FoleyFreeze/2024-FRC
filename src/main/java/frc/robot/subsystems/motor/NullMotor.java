package frc.robot.subsystems.motor;

import edu.wpi.first.math.geometry.Rotation2d;

public class NullMotor implements Motor{

    public NullMotor(MotorCal cal){
        
    }

    @Override
    public void setPower(double power) {
        
    }

    @Override
    public double getPosition() {
        return 0;
    }
    
    @Override
    public void setPosition(double position) {
        
    }

    @Override
    public void setEncoderPosition(double position) {
        
    }

    @Override
    public void setVelocity(double rpm) {
        
    }

    @Override
    public double getCurrent() {
        return 0;
    }

    @Override
    public double getTemp(){
        return 0;
    }

    @Override
    public void setBrakeMode(boolean brakeMode) {
        
    }

    @Override
    public void setPIDPwrLim(double pwrLim) {
        
    }
    
    @Override
    public void setPIDPwrLim(double pwrLimPos, double pwrLimNeg) {
        
    }

    @Override 
    public Rotation2d getRotation(){
        return new Rotation2d(0);
    }

    @Override 
    public void setRotation(Rotation2d angle){
        
    }
}
