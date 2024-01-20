package frc.robot.auton;

import java.util.Optional;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class Auton {
    
    public static Command getChoreoPath(String fileName, RobotContainer r){
        ChoreoTrajectory traj = Choreo.getTrajectory(fileName);

        return Choreo.choreoSwerveCommand(
            traj, 
            r.drive::getPose,
            new PIDController(0.0, 0.0, 0.0),   //X Controller (units are meters)
            new PIDController(0.0, 0.0, 0.0),   //Y Controller
            new PIDController(0.0, 0.0, 0.0),   //Rotate Controller (units are radians)
            (ChassisSpeeds speeds) ->
                r.drive.swerveDrivePwr(speeds, true),
            () -> {   
                Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == Alliance.Red;
            },
            r.drive
        );
    }
}


