// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.cals.ClimberCals;
import frc.robot.cals.DriveCals;
import frc.robot.cals.GatherCals;
import frc.robot.cals.InputsCals;
import frc.robot.cals.ShooterCals;
import frc.robot.cals.SlappahCals;
import frc.robot.cals.VisionCals;
import frc.robot.commands.climber.CmdClimb;
import frc.robot.commands.drive.CmdDrive;
import frc.robot.commands.drive.CmdDriveNoteTraj;
import frc.robot.commands.drive.CmdDriveToNote;
import frc.robot.commands.gather.CmdGather;
import frc.robot.commands.shooter.CMDShoot;
import frc.robot.commands.slappah.CmdTransfer;
import frc.robot.subsystems.RoboState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gather.Gather;
import frc.robot.subsystems.inputs.Inputs;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.slappah.Slappah;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {

  public Climber climber;
  public Drive drive;
  public Gather gather;
  public Shooter shooter;
  public Slappah slappah;
  public Inputs inputs;
  public Vision vision;
  public Lights lights;
  public RoboState state;

  
  public enum AutonType{
    DO_NOTHING, PREGEN, DENIAL, SELECTABLE, TEST
  }

  private LoggedDashboardChooser<AutonType> autoChooser = new LoggedDashboardChooser<>("AutonType");
  private LoggedDashboardChooser<Integer> notePriorityA = new LoggedDashboardChooser<>("A Note Priority");
  private LoggedDashboardChooser<Integer> notePriorityB = new LoggedDashboardChooser<>("B Note Priority");
  private LoggedDashboardChooser<Integer> notePriorityC = new LoggedDashboardChooser<>("C Note Priority");
  private LoggedDashboardChooser<Integer> notePriorityD = new LoggedDashboardChooser<>("D Note Priority");
  private LoggedDashboardChooser<Integer> notePriorityE = new LoggedDashboardChooser<>("E Note Priority");
  private LoggedDashboardChooser<Integer> notePriorityF = new LoggedDashboardChooser<>("F Note Priority");
  private LoggedDashboardChooser<Integer> notePriorityG = new LoggedDashboardChooser<>("G Note Priority");
  private LoggedDashboardChooser<Integer> notePriorityH = new LoggedDashboardChooser<>("H Note Priority");
  private LoggedDashboardChooser<Integer> totalNotes = new LoggedDashboardChooser<>("Total Notes"); //stop after this many notes

  public RobotContainer() {
    inputs = new Inputs(this, new InputsCals());
    drive = new Drive(this, new DriveCals());
    gather = new Gather(this, new GatherCals());
    vision = new Vision(this, new VisionCals());
    shooter = new Shooter(this, new ShooterCals());
    slappah = new Slappah(this, new SlappahCals());
    climber = new Climber(this, new ClimberCals());
    lights = new Lights();
    state = new RoboState();
    configureBindings();

    //log path data with advantage scope
    PathPlannerLogging.setLogActivePathCallback((path) -> {
      //TODO: idk if this works how we think it does
      for(Pose2d p : path){
        Logger.recordOutput("PathPlanner/ActivePath", p);
      }
    });
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> Logger.recordOutput("PathPlanner/CurrentPose", pose));
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> Logger.recordOutput("PathPlanner/TargetPose", pose));

    //TODO: fill in auto choosers
    autoChooser.addDefaultOption("Do Nothing", AutonType.DO_NOTHING);
    autoChooser.addOption("PREGEN", AutonType.PREGEN);

    Shuffleboard.getTab("Auton").add(autoChooser.getSendableChooser()).withPosition(0, 0);
  }

  private void configureBindings() {
    drive.setDefaultCommand(new CmdDrive(this).ignoringDisable(true));

    inputs.resetSwerveZerosTRIM2DN.onTrue(new InstantCommand(drive::learnSwerveOffsets).ignoringDisable(true));
    inputs.resetFieldOrientedLTRIM.onTrue(new InstantCommand(drive::resetFieldOrientedAngle).ignoringDisable(true));
    inputs.resetFieldOdometryRTRIM.onTrue(new InstantCommand(drive::resetFieldOdometry).ignoringDisable(true));

    //gather commands
    inputs.gatherTriggerSWE.and(inputs.cameraEnableSWD).whileTrue(CmdGather.gather(this).deadlineWith(new CmdDriveNoteTraj(this)).ignoringDisable(true));
    inputs.gatherTriggerSWE.and(inputs.cameraEnableSWD.negate()).whileTrue(CmdGather.gather(this));

    //shoot commands
    inputs.shootTriggerSWH
        .and(inputs.cameraEnableSWD.negate())
        .and(state.isPrimeT.negate())
        .and(inputs.SWBHi.negate())
        .and(inputs.SWBLo.negate())
        .onTrue(CMDShoot.fixedPrime(this));

    inputs.shootTriggerSWH
        .and(inputs.cameraEnableSWD.negate())
        .and(state.isPrimeT)
        .and(inputs.SWBHi.negate())
        .and(inputs.SWBLo.negate())
        .onTrue(CMDShoot.simpleShoot(this));

    //climber commands
    inputs.shootTriggerSWH.and(inputs.SWBHi.or(inputs.SWBLo)).whileTrue(CmdClimb.testClimb(this));

    //TODO: map here for now
    inputs.SWC.and(inputs.SWBHi.negate().and(inputs.SWBLo.negate())).whileTrue(CmdGather.unGather(this));
    //if in climb mode, ungather will transfer
    inputs.SWC.and(inputs.SWBHi.or(inputs.SWBLo)).onTrue(CmdTransfer.transferForAmp(this));

    //SmartDashboard.putData("TestCmd", new RunCommand(() -> {shooter.setShootPower(0.12); slappah.setTransferPower(-0.6); gather.setGatePower(0.8);}).raceWith(new WaitCommand(10)).finallyDo(() -> {shooter.setShootPower(0); slappah.setTransferPower(0); gather.setGatePower(0);}));

    //TODO: uncomment once there is a control board
    //inputs.shift.negate().and(inputs.shootAngleJogUp).onTrue(new InstantCommand(() -> shooter.jogAngle(shooter.k.jogAngleIncriment)));
    //inputs.shift.negate().and(inputs.shootAngleJogDn).onTrue(new InstantCommand(() -> shooter.jogAngle(-shooter.k.jogAngleIncriment)));
    //inputs.shift.and(inputs.shootAngleJogUp).onTrue(new InstantCommand(() -> shooter.jogSpeed(shooter.k.jogSpeedIncriment)));
    //inputs.shift.and(inputs.shootAngleJogDn).onTrue(new InstantCommand(() -> shooter.jogSpeed(-shooter.k.jogSpeedIncriment)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
  
  private Command autonCommand;
  private String lastSelectedAuton = "";
  
  public void determineAuton(){
    String selectedAuton = "";
    selectedAuton += autoChooser.get().ordinal();
    selectedAuton += notePriorityA.get();
    selectedAuton += notePriorityB.get();
    selectedAuton += notePriorityC.get();
    selectedAuton += notePriorityD.get();
    selectedAuton += notePriorityE.get();
    selectedAuton += notePriorityF.get();
    selectedAuton += notePriorityG.get();
    selectedAuton += notePriorityH.get();
    selectedAuton += totalNotes.get();

    if (!selectedAuton.equals(lastSelectedAuton)){
      switch(autoChooser.get()){
        case DENIAL:
          autonCommand = new InstantCommand();
          break;
        case PREGEN:
          autonCommand = new InstantCommand();
          break;
        case SELECTABLE:
          autonCommand = new InstantCommand();
          break;
        case TEST:
          autonCommand = new InstantCommand();
          break;

        case DO_NOTHING:
        default:
          autonCommand = new InstantCommand();
          break;
      }
    }
  }
}
