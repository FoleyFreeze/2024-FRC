// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auton.ChoreoAuto;
import frc.robot.auton.CmdAuton;
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

  private LoggedDashboardChooser<AutonType> autoChooser;
  private LoggedDashboardChooser<Integer> notePriorityA;
  private LoggedDashboardChooser<Integer> notePriorityB;
  private LoggedDashboardChooser<Integer> notePriorityC;
  private LoggedDashboardChooser<Integer> notePriorityD;
  private LoggedDashboardChooser<Integer> notePriorityE;
  private LoggedDashboardChooser<Integer> notePriorityF;
  private LoggedDashboardChooser<Integer> notePriorityG;
  private LoggedDashboardChooser<Integer> notePriorityH;
  private LoggedDashboardChooser<Integer> totalNotes;

  public RobotContainer() {
    inputs = new Inputs(this, new InputsCals());
    drive = new Drive(this, new DriveCals());
    gather = new Gather(this, new GatherCals());
    vision = new Vision(this, new VisionCals());
    shooter = new Shooter(this, new ShooterCals());
    slappah = new Slappah(this, new SlappahCals());
    climber = new Climber(this, new ClimberCals());
    lights = new Lights(this);
    state = new RoboState();

    configureBindings();

    configureAutonSelection();

    configurePathPlanning();
    
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
    return autonCommand;
  }
  
  private Command autonCommand = new InstantCommand();
  private String lastSelectedAuton = "";
  private boolean selectedAutonGenerated = false;
  
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
      lastSelectedAuton = selectedAuton;
      switch(autoChooser.get()){
        //TODO: include totalNotes in the pregen path autos to stop early
        //Probably do this as part of a AutonMonitor running in parallel
        case DENIAL:
          autonCommand = new InstantCommand();
          break;

        case PREGEN:
          autonCommand = ChoreoAuto.getPregen3NoteMid("3NoteMid", this);
          break;

        case SELECTABLE:
          autonCommand = CmdAuton.selectedAuto(this,
                                notePriorityA.get(),
                                notePriorityB.get(),
                                notePriorityC.get(),
                                notePriorityD.get(),
                                notePriorityE.get(),
                                notePriorityF.get(),
                                notePriorityG.get(),
                                notePriorityH.get(),
                                totalNotes.get()
          );
          break;

        case TEST:
          //autonCommand = ChoreoAuto.getPathPlannerAuto("TestStraight", this);
          //autonCommand = ChoreoAuto.getPathPlannerAuto("TestArc", this);
          autonCommand = ChoreoAuto.getChoreoPath("TestArcSpin", this);
          break;

        case DO_NOTHING:
        default:
          autonCommand = new InstantCommand();
          break;
      }

      System.out.println("Generated Auton");
    }
  }

  private void configurePathPlanning(){
    //log path data with advantage scope
    PathPlannerLogging.setLogActivePathCallback((path) -> {
        Logger.recordOutput("PathPlanner/ActivePath", path.toArray(new Pose2d[path.size()]));
    });
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> Logger.recordOutput("PathPlanner/CurrentPose", pose));
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> Logger.recordOutput("PathPlanner/TargetPose", pose));

    //configure auto builder
    AutoBuilder.configureHolonomic(
            drive::getPose, // Robot pose supplier
            drive::resetFieldOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            drive::getRelVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            drive::swerveDriveVel, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(11.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(11.0, 0.0, 0.0), // Rotation PID constants
                    drive.k.maxWheelSpeed, // Max module speed, in m/s
                    drive.k.wheelBR.wheelLocation.getNorm(), // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            drive // Reference to this subsystem to set requirements
    );
  }

  private void configureAutonSelection(){
    autoChooser = new LoggedDashboardChooser<>("AutonType");
    notePriorityA = new LoggedDashboardChooser<>("A Note Priority");
    notePriorityB = new LoggedDashboardChooser<>("B Note Priority");
    notePriorityC = new LoggedDashboardChooser<>("C Note Priority");
    notePriorityD = new LoggedDashboardChooser<>("D Note Priority");
    notePriorityE = new LoggedDashboardChooser<>("E Note Priority");
    notePriorityF = new LoggedDashboardChooser<>("F Note Priority");
    notePriorityG = new LoggedDashboardChooser<>("G Note Priority");
    notePriorityH = new LoggedDashboardChooser<>("H Note Priority");
    totalNotes = new LoggedDashboardChooser<>("Total Notes"); //stop after this many notes

    autoChooser.addDefaultOption("Do Nothing", AutonType.DO_NOTHING);
      autoChooser.addOption("Pregenerated", AutonType.PREGEN);
      autoChooser.addOption("Denial", AutonType.DENIAL);
      autoChooser.addOption("Selectable", AutonType.SELECTABLE);
      autoChooser.addOption("Test", AutonType.TEST);
    
    addNoteOrderHelper(notePriorityA);
    addNoteOrderHelper(notePriorityB);
    addNoteOrderHelper(notePriorityC);
    addNoteOrderHelper(notePriorityD);
    addNoteOrderHelper(notePriorityE);
    addNoteOrderHelper(notePriorityF);
    addNoteOrderHelper(notePriorityG);
    addNoteOrderHelper(notePriorityH);
    addNoteOrderHelper(totalNotes);
    totalNotes.addDefaultOption("0", 0);

    ShuffleboardTab autoTab = Shuffleboard.getTab("Auton");
    autoTab.add(autoChooser.getSendableChooser()).withPosition(0, 0);
    autoTab.add(totalNotes.getSendableChooser()).withPosition(1, 0);
    autoTab.add(notePriorityA.getSendableChooser()).withPosition(1, 1);
    autoTab.add(notePriorityB.getSendableChooser()).withPosition(2, 1);
    autoTab.add(notePriorityC.getSendableChooser()).withPosition(3, 1);
    autoTab.add(notePriorityD.getSendableChooser()).withPosition(4, 1);
    autoTab.add(notePriorityE.getSendableChooser()).withPosition(5, 1);
    autoTab.add(notePriorityF.getSendableChooser()).withPosition(1, 2);
    autoTab.add(notePriorityG.getSendableChooser()).withPosition(2, 2);
    autoTab.add(notePriorityH.getSendableChooser()).withPosition(3, 2);
  }

  private void addNoteOrderHelper(LoggedDashboardChooser<Integer> c){
    c.addDefaultOption("N/A", 0);
    c.addOption("1", 1);
    c.addOption("2", 2);
    c.addOption("3", 3);
    c.addOption("4", 4);
    c.addOption("5", 5);
    c.addOption("6", 6);
    c.addOption("7", 7);
    c.addOption("8", 8);
  }
}
