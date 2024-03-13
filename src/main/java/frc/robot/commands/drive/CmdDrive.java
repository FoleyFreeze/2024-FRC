package frc.robot.commands.drive;

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

        pidController = new ProfiledPIDController(0.15, 0, 0, new Constraints(2, 2));
        pidController.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    @Override
    public void execute(){
        ChassisSpeeds speed = r.inputs.getChassisSpeeds();
        if(r.state.climbDeploy != ClimbState.NONE){
            speed.times(0.2);
            
            //init setpoint 
            if(!inAngleControl){
                switch(r.inputs.getClimbDir()){
                    case 1://left
                        angleSetpoint = Rotation2d.fromDegrees(120);
                    break;
                    case 2://right
                        angleSetpoint = Rotation2d.fromDegrees(-120);
                    break;
                    case 0://back
                        angleSetpoint = Rotation2d.fromDegrees(180);
                    break;
                }
                if(DriverStation.getAlliance().get() == Alliance.Red){
                    angleSetpoint.plus(Rotation2d.fromDegrees(180));
                }
            }

            if(Math.abs(speed.omegaRadiansPerSecond) > 0.02){
                //modify setpoint
                angleSetpoint = r.drive.getAngle();
            } else { 
                //control to setpoint
                double measurement = MathUtil.angleModulus(r.drive.getAngle().getRadians());
                double goal = MathUtil.angleModulus(angleSetpoint.getRadians());
                speed.omegaRadiansPerSecond = pidController.calculate(measurement, goal);
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
