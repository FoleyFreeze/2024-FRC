package frc.robot.commands.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RoboState.ClimbState;

public class CmdDrive extends Command {
    
    RobotContainer r;

    boolean inAngleControl = false;
    Rotation2d angleSetpoint;

    ProfiledPIDController pidController;

    public CmdDrive(RobotContainer r){
        this.r = r;
        addRequirements(r.drive);

        pidController = new ProfiledPIDController(1, 0, 0, new Constraints(2, 2));
        pidController.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    @Override
    public void execute(){
        ChassisSpeeds speed = r.inputs.getChassisSpeeds();
        if(r.state.climbDeploy != ClimbState.NONE){
            speed.times(0.2);
            
            //init setpoint 
            if(!inAngleControl){
                inAngleControl = true;
                switch(r.inputs.getClimbDir()){
                    case 1://left
                        angleSetpoint = Rotation2d.fromDegrees(120);
                    break;
                    case -1://right
                        angleSetpoint = Rotation2d.fromDegrees(-120);
                    break;
                    case 0://back
                        angleSetpoint = Rotation2d.fromDegrees(180);
                    break;
                    default:
                        angleSetpoint = new Rotation2d();
                        System.out.println("Unknown climb dir: " + r.inputs.getClimbDir());
                }
                if(DriverStation.getAlliance().get() == Alliance.Red){
                    angleSetpoint.plus(Rotation2d.fromDegrees(180));
                }

                double measurement = MathUtil.angleModulus(r.drive.getAngle().getRadians());
                pidController.reset(measurement);
            }

            if(Math.abs(speed.omegaRadiansPerSecond) > 0.02){
                //modify setpoint
                angleSetpoint = r.drive.getAngle();
                double measurement = MathUtil.angleModulus(r.drive.getAngle().getRadians());
                pidController.reset(measurement);
            } else { 
                //control to setpoint
                double measurement = MathUtil.angleModulus(r.drive.getAngle().getRadians());
                double goal = MathUtil.angleModulus(angleSetpoint.getRadians());
                speed.omegaRadiansPerSecond = pidController.calculate(measurement, goal);
                speed.omegaRadiansPerSecond = MathUtil.clamp(speed.omegaRadiansPerSecond, -0.3, 0.3);

                Logger.recordOutput("Drive/AnglePID/Goal", goal);
                Logger.recordOutput("Drive/AnglePID/Setpoint", pidController.getSetpoint().position);
                Logger.recordOutput("Drive/AnglePID/Measurement", measurement);
            }

        } else {
            inAngleControl = false;
        }
        r.drive.swerveDrivePwr(speed, r.inputs.fieldOrientedSWA.getAsBoolean() && r.state.climbDeploy == ClimbState.NONE);    
    }

    @Override
    public void end(boolean interrupted){
        r.drive.swerveDrivePwr(new ChassisSpeeds(0, 0 , 0), false);
    }
}
