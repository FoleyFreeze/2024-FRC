package frc.robot.commands.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.auton.CmdAuton;
import frc.robot.auton.Locations;
import frc.robot.subsystems.RoboState.ClimbState;

public class CmdDrive extends Command {
    
    RobotContainer r;

    boolean inClimbAngleControl = false;
    boolean inPodiumAngleControl = false;
    boolean inCameraAngleControl = false;

    Rotation2d podiumAngleBlue = Rotation2d.fromDegrees(-30 + 4.5);
    Rotation2d podiumAngleRed =  Rotation2d.fromDegrees(30  + 180 + 4.5);
    double driverOverrideTime = 0.25;//seconds

    Rotation2d angleSetpoint;

    ProfiledPIDController pidController;

    Timer timeSinceDriverRotate;

    double initVelTol = 0.5;
    double initPosTol = Math.toRadians(5);

    public CmdDrive(RobotContainer r){
        this.r = r;
        addRequirements(r.drive);

        pidController = new ProfiledPIDController(2, 0, 0.05, new Constraints(7, 4));
        pidController.enableContinuousInput(-Math.PI, Math.PI);
        pidController.setTolerance(initPosTol, initVelTol); //3deg error and 0.5 rad/s

        timeSinceDriverRotate = new Timer();
        timeSinceDriverRotate.restart();
    }
    
    @Override
    public void execute(){
        r.state.autoDrive = false;

        ChassisSpeeds speed = r.inputs.getChassisSpeeds();
        if(r.state.climbDeploy != ClimbState.NONE){
            pidController.setTolerance(initPosTol, initVelTol);

            //give the driver reversed robot oriented drive
            speed.times(0.2);
            //speed.omegaRadiansPerSecond *= -1;
            
            //init setpoint 
            if(!inClimbAngleControl){
                inClimbAngleControl = true;
                switch(r.inputs.getClimbDir()){
                    case -1://left
                        angleSetpoint = Rotation2d.fromDegrees(120-180);
                    break;
                    case 1://right
                        angleSetpoint = Rotation2d.fromDegrees(-120-180); //was also -180 at the end 3/20
                    break;
                    case 0://back
                        angleSetpoint = Rotation2d.fromDegrees(180);
                    break;
                    default:
                        angleSetpoint = new Rotation2d();
                        System.out.println("Unknown climb dir: " + r.inputs.getClimbDir());
                }
                if(DriverStation.getAlliance().get() == Alliance.Red){
                    angleSetpoint = angleSetpoint.plus(Rotation2d.fromDegrees(180));
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

            inPodiumAngleControl = false;
            inCameraAngleControl = false;

        } else if(r.state.isPrime && r.inputs.getFixedTarget() == 2){
            //if we are priming for a podium shot, go to the right angle
            pidController.setTolerance(initPosTol, initVelTol);

            if(!inPodiumAngleControl){
                inPodiumAngleControl = true; 
                if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
                    angleSetpoint = podiumAngleRed;
                } else {
                    angleSetpoint = podiumAngleBlue;
                }

                double measurement = MathUtil.angleModulus(r.drive.getAngle().getRadians());
                pidController.reset(measurement);
            }

            if(Math.abs(speed.omegaRadiansPerSecond) > 0.05){
                double measurement = MathUtil.angleModulus(r.drive.getAngle().getRadians());
                pidController.reset(measurement);

                timeSinceDriverRotate.restart();
            } else if(!timeSinceDriverRotate.hasElapsed(driverOverrideTime)){
                double measurement = MathUtil.angleModulus(r.drive.getAngle().getRadians());
                pidController.reset(measurement);

            } else {
                //run the pid

                double measurement = MathUtil.angleModulus(r.drive.getAngle().getRadians());
                double goal = MathUtil.angleModulus(angleSetpoint.getRadians());
                speed.omegaRadiansPerSecond = pidController.calculate(measurement, goal);
                speed.omegaRadiansPerSecond = MathUtil.clamp(speed.omegaRadiansPerSecond, -0.3, 0.3);

                Logger.recordOutput("Drive/AnglePID/Goal", goal);
                Logger.recordOutput("Drive/AnglePID/Setpoint", pidController.getSetpoint().position);
                Logger.recordOutput("Drive/AnglePID/Measurement", measurement);
                Logger.recordOutput("Drive/AnglePID/ErrorDeg", Math.toDegrees(pidController.getPositionError()));
                Logger.recordOutput("Drive/AnglePID/VelocityErr", pidController.getVelocityError());
                
            }

            inClimbAngleControl = false;
            inCameraAngleControl = false;

        } else if(r.state.isPrime && (r.inputs.SWBHi.getAsBoolean() || DriverStation.isAutonomous())){
            //if camera aiming, aim at the speaker

            if(!inCameraAngleControl){
                inCameraAngleControl = true; 
                
                //since robot must point backwards at speaker, draw a line from speaker to robot and point along it
                angleSetpoint = r.drive.getPose().getTranslation().minus(Locations.tagSpeaker).getAngle().plus(CmdAuton.shooterOffset);

                double measurement = MathUtil.angleModulus(r.drive.getAngle().getRadians());
                pidController.reset(measurement);
                r.drive.atAngleSetpoint = pidController.atGoal();
            }

            if(Math.abs(speed.omegaRadiansPerSecond) > 0.05){
                double measurement = MathUtil.angleModulus(r.drive.getAngle().getRadians());
                pidController.reset(measurement);
                r.drive.atAngleSetpoint = pidController.atGoal();

                timeSinceDriverRotate.restart();
            } else if(!timeSinceDriverRotate.hasElapsed(driverOverrideTime)){
                double measurement = MathUtil.angleModulus(r.drive.getAngle().getRadians());
                pidController.reset(measurement);
                r.drive.atAngleSetpoint = pidController.atGoal();

            } else {
                //run the pid
                double measurement = MathUtil.angleModulus(r.drive.getAngle().getRadians());
                double goal;
                if(r.inputs.getFixedTarget() == 1){
                    //if lob mode
                    double dist = r.drive.getPose().getTranslation().getDistance(Locations.tagSpeaker);
                    Rotation2d angleOffset = new Rotation2d(r.shooter.getLobOffset(dist));
                    Translation2d targetPosition = Locations.tagAmp.plus(Locations.tagSpeaker).div(2);
                    goal = r.drive.getPose().getTranslation().minus(targetPosition).getAngle().plus(angleOffset).getRadians();

                    double angleTol = Math.toRadians(7);//7deg? too much?
                    pidController.setTolerance(angleTol, initVelTol);
                } else {
                    //if shoot mode
                    Translation2d vecToSpeaker = r.drive.getPose().getTranslation().minus(Locations.tagSpeaker);
                    goal = vecToSpeaker.getAngle().plus(CmdAuton.shooterOffset).getRadians();

                    double speakerDist = vecToSpeaker.getNorm();
                    double angleTol = r.shooter.getBotAngleTol(speakerDist);
                    pidController.setTolerance(angleTol, initVelTol);
                }
                Logger.recordOutput("Shooter/AutoAimAngle", Math.toDegrees(goal));

                speed.omegaRadiansPerSecond = pidController.calculate(measurement, goal);
                speed.omegaRadiansPerSecond = MathUtil.clamp(speed.omegaRadiansPerSecond, -0.3, 0.3);

                r.drive.atAngleSetpoint = pidController.atGoal();

                Logger.recordOutput("Drive/AnglePID/Goal", goal);
                Logger.recordOutput("Drive/AnglePID/Setpoint", pidController.getSetpoint().position);
                Logger.recordOutput("Drive/AnglePID/Measurement", measurement);
                Logger.recordOutput("Drive/AnglePID/ErrorDeg", Math.toDegrees(pidController.getPositionError()));
                Logger.recordOutput("Drive/AnglePID/VelocityErr", pidController.getVelocityError());
                //Logger.recordOutput("Drive/AnglePID/Velocity", );
                Logger.recordOutput("Drive/AnglePID/Power", speed.omegaRadiansPerSecond);

                if(r.inputs.shootTriggerSWH.getAsBoolean()){
                    //once we are priming and the trigger is still held we disable human driving
                    speed.vxMetersPerSecond = 0;
                    speed.vyMetersPerSecond = 0;
                }
            }
            
            inClimbAngleControl = false;
            inPodiumAngleControl = false;

        } else {
            inClimbAngleControl = false;
            inPodiumAngleControl = false;
            inCameraAngleControl = false;
        }

        r.drive.swerveDrivePwr(speed, r.inputs.fieldOrientedSWA.getAsBoolean()/*  && r.state.climbDeploy == ClimbState.NONE*/);    
    }

    @Override
    public void end(boolean interrupted){
        r.state.autoDrive = true;//assume we are being interrupted for autodrive
        r.drive.swerveDrivePwr(new ChassisSpeeds(), false);
    }
}
