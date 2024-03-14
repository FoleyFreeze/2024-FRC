package frc.robot.commands.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class CmdDriveFixedAngle extends Command{
    RobotContainer r;

    ProfiledPIDController pidController;

    Supplier<Rotation2d> angleSupplier;

    public CmdDriveFixedAngle(RobotContainer r, Supplier<Rotation2d> angleSupplier){
        this.r = r;
        this.angleSupplier = angleSupplier;
        addRequirements(r.drive);

        pidController = new ProfiledPIDController(0.3, 0, 0, new Constraints(2, 2));
        pidController.enableContinuousInput(-Math.PI, Math.PI);
        pidController.setTolerance(Math.toRadians(5));
    }

    @Override
    public void initialize(){
        double measurement = MathUtil.angleModulus(r.drive.getAngle().getRadians());
        pidController.reset(measurement);

        Rotation2d angle = angleSupplier.get();
        double goal = MathUtil.angleModulus(angle.getRadians());
        pidController.setGoal(goal);

        Logger.recordOutput("Drive/AnglePID/Goal", goal);
    }
    
    @Override
    public void execute(){
        ChassisSpeeds speed = r.inputs.getChassisSpeeds();
        double measurement = MathUtil.angleModulus(r.drive.getAngle().getRadians());
        

        if(Math.abs(speed.omegaRadiansPerSecond) > 0.1){
            //modify setpoint
            pidController.reset(measurement);
            double goal = MathUtil.angleModulus(r.drive.getAngle().getRadians());
            pidController.setGoal(goal);
            
            Logger.recordOutput("Drive/AnglePID/Goal", goal);
        } else { 
            //control to setpoint
            speed.omegaRadiansPerSecond = pidController.calculate(measurement);
            speed.omegaRadiansPerSecond = MathUtil.clamp(speed.omegaRadiansPerSecond, -0.3, 0.3);
        }

        Logger.recordOutput("Drive/AnglePID/Setpoint", pidController.getSetpoint().position);
        Logger.recordOutput("Drive/AnglePID/Measurement", measurement);
    }

    @Override
    public void end(boolean interrupted){
        r.drive.swerveDrivePwr(new ChassisSpeeds(0, 0 , 0), false);
    }

    public boolean atAngle(){
        return pidController.atGoal();
    }
}
