// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auton.LocalADStarAK;
import frc.robot.util.CommandLogger;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    private static final boolean simOnly = false;

    @Override
    public void robotInit() {
        Pathfinding.setPathfinder(new LocalADStarAK());

        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME); // Set a metadata value
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GIT_SHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch(BuildConstants.DIRTY){
          case 0:
            Logger.recordMetadata("GitDirty", "All Changes Committed");
            break;
          case 1:
            Logger.recordMetadata("GitDirty", "Uncomitted changes");
            break;
          default:
            Logger.recordMetadata("GitDirty", "Unknown");
            break;
        }

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(21, ModuleType.kRev); // Enables power distribution logging
        } else {
            //if simulating, determine if we want a replay or sim mode
            if(simOnly){
                Logger.addDataReceiver(new NT4Publisher());
            } else {
                setUseTiming(false); // Run as fast as possible
                //replay the log file
                String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
                Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
            }
        }

        // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
        
        //record every active command to the log
        CommandLogger.logScheduler();
        
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_robotContainer.state.periodic();
    }

    @Override
    public void disabledInit() {
        m_robotContainer.slappah.setBrake(false);
        m_robotContainer.drive.setSwerveBrake(false);
    }

    @Override
    public void disabledPeriodic() {
        m_robotContainer.determineAuton();
    }

    @Override
    public void disabledExit() {
        m_robotContainer.slappah.setBrake(true);
    }

    @Override
    public void autonomousInit() {
        //always init with brakes off
        m_robotContainer.drive.setBrake(false);
        m_robotContainer.drive.setSwerveBrake(true);
        m_robotContainer.climber.setBrakes(true);

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        //go to brake after auto in case we are 
        //coasting into a wall or something
        m_robotContainer.drive.setBrake(true);
        m_robotContainer.shooter.setShootPower(0);//turn off shooter after auto
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        //always init with brakes off
        m_robotContainer.drive.setBrake(false);
        m_robotContainer.drive.setSwerveBrake(true);
        m_robotContainer.climber.setBrakes(true);
        m_robotContainer.climber.captureSetpoints();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
