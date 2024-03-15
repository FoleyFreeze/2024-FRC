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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auton.ChoreoAuto;
import frc.robot.auton.CmdAuton;
import frc.robot.auton.Locations;
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
import frc.robot.subsystems.RoboState.ClimbState;
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

  public enum StartLocationType{
    AMP_SIDE, SPEAKER_CENTER, SOURCE_SIDE, APRILTAG_0Deg, APRILTAG
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
  private LoggedDashboardChooser<StartLocationType> startChooser;
  private LoggedDashboardChooser<Integer> waitChooser;

  public RobotContainer() {
    Locations.loadTagData();

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
    gather.setDefaultCommand(CmdGather.backwardsGather(this));

    inputs.resetSwerveZerosTRIM2DN.onTrue(new InstantCommand(drive::learnSwerveOffsets).ignoringDisable(true));
    inputs.resetFieldOrientedLTRIM.onTrue(new InstantCommand(drive::resetFieldOrientedAngle).ignoringDisable(true));
    inputs.resetFieldOdometryRTRIM.onTrue(new InstantCommand(drive::resetFieldOdometry).ignoringDisable(true));
    inputs.resetArmLRTRIMR.onTrue(new InstantCommand(slappah::resetArmAngle).ignoringDisable(true));
    inputs.resetShooterLRTRIML.onTrue(new InstantCommand(shooter::resetAngle).ignoringDisable(true));

    //gather commands
    inputs.gatherTriggerSWE
        .and(inputs.cameraEnableSWD)
        .and(state.climbDeployT.negate())
        .and(state.hasTransferT.negate())
        .whileTrue(CmdGather.gather(this).deadlineWith(new CmdDriveNoteTraj(this)).ignoringDisable(true));
    
    inputs.gatherTriggerSWE
        .and(inputs.cameraEnableSWD.negate())
        .and(state.climbDeployT.negate())
        .and(state.hasTransferT.negate())
        .whileTrue(CmdGather.gather(this));

    inputs.gatherTriggerSWE
        .and(state.hasTransferT)
        .and(state.climbDeployT.negate())
        .onTrue(CmdTransfer.goToPreAmp(this));

    //shoot commands
    /*inputs.shootTriggerSWH
        .and(inputs.cameraEnableSWD.negate())
        .and(state.isPrimeT.negate())
        .and(inputs.SWBHi.negate())
        .and(inputs.SWBLo.negate())
        .onTrue(CMDShoot.fixedPrime(this));*/

    inputs.shootTriggerSWH
        //.and(inputs.cameraEnableSWD.negate()) //TODO: add when camera shoot exists
        //.and(state.hasNoteT)
        .and(state.climbDeployT.negate())
        .and(state.hasTransferT.negate())
        //.and(inputs.SWBHi.negate())
        //.and(inputs.SWBLo.negate())
        .onTrue(CMDShoot.simpleShoot(this));
        //.onTrue(CMDShoot.simpleAmpShoot(this)); //for testing, should move to SWC eventually

    //climb commands
    inputs.shootTriggerSWH
        .and(state.hasTransferT)
        .and(state.climbDeployT.negate())
        .onTrue(CmdTransfer.scoreInAmp(this));

    inputs.shootTriggerSWH
        .and(new Trigger(() -> state.climbDeploy == ClimbState.DEPLOYED))
        .onTrue(CmdClimb.hook(this));

    inputs.shootTriggerSWH
        .and(new Trigger(() -> state.climbDeploy == ClimbState.HOOKED))
        .onTrue(CmdClimb.climb(this));

    inputs.shootTriggerSWH
        .and(new Trigger(()-> state.climbDeploy == ClimbState.CLIMBED))
        .whileTrue(CmdClimb.shootTrap(this));

    //unshooot trap
    inputs.gatherTriggerSWE
        .and(new Trigger(() -> state.climbDeploy == ClimbState.CLIMBED))
        .whileTrue(CmdClimb.unshootTrap(this));

    

    //control board commands
    //transfer to arm
    inputs.transferB3
        .and(inputs.shiftB6.negate())
        //.and(state.hasNoteT)
        .and(state.hasTransferT.negate())
        .and(state.climbDeployT.negate())
        .onTrue(CmdTransfer.transferForAmp(this, inputs.transferB3));

    //untransfer from arm
    inputs.transferB3
        .and(inputs.shiftB6)
        //.and(state.hasNoteT)
        .and(state.hasTransferT)
        .and(state.climbDeployT.negate())
        .onTrue(CmdTransfer.unTransferFull(this, inputs.transferB3));

    //deploy climb
    inputs.climbDeployB4
        .and(inputs.shiftB6.negate())
        .and(state.climbDeployT.negate())
        .and(state.hasTransferT.negate())
        .onTrue(CmdClimb.deployClimb(this));

    //undeploy climb
    inputs.climbDeployB4
        .and(inputs.shiftB6)
        .and(new Trigger(() -> state.climbDeploy == ClimbState.DEPLOYED))
        .and(state.hasTransferT.negate())
        .onTrue(CmdClimb.undeployClimb(this));

    inputs.climbWinchB2
        .and(state.climbDeployT.or(new Trigger(()->slappah.inputs.anglePosition > 45)))
        .and(inputs.shiftB6.negate())
        .whileTrue(new RunCommand(climber::winchUp,climber)
            .finallyDo(climber::winchHold));

    inputs.climbWinchB2
        .and(state.climbDeployT.or(new Trigger(()->slappah.inputs.anglePosition > 45)))
        .and(inputs.shiftB6)
        .whileTrue(new RunCommand(climber::winchDown,climber)
            .finallyDo(climber::winchHold));

    //gather
    inputs.gatherBtnB5
        .and(inputs.shiftB6.negate())
        .and(state.climbDeployT.negate())
        .and(state.hasTransferT.negate())
        .whileTrue(CmdGather.gather(this));

    //ungather
    inputs.gatherBtnB5
        .and(inputs.shiftB6)
        .and(state.climbDeployT.negate())
        .and(state.hasTransferT.negate())
        .whileTrue(CmdGather.unGather(this));

    //shoot
    inputs.shootBtnB1
        .and(inputs.shiftB6.negate())
        .and(state.climbDeployT.negate())
        .and(state.hasTransferT)
        .onTrue(CmdTransfer.scoreInAmp(this));

    //coast winch motors
    inputs.shiftB6
        .and(inputs.climbDeployB4)
        .and(inputs.climbWinchB2)
        .and(new Trigger(() -> DriverStation.isDisabled() && !DriverStation.isFMSAttached()))
        .onTrue(new InstantCommand(() -> climber.setBrakes(false)).ignoringDisable(true)
            .andThen(new PrintCommand("Winches in Coast Mode")));

    inputs.shiftB6.negate()
        .and(inputs.shootAngleJogUp)
        .onTrue(new InstantCommand(() -> shooter.jogAngle(shooter.k.jogAngleIncriment)));

    inputs.shiftB6.negate()
        .and(inputs.shootAngleJogDn)
        .onTrue(new InstantCommand(() -> shooter.jogAngle(-shooter.k.jogAngleIncriment)));

    inputs.shiftB6
        .and(inputs.shootAngleJogUp)
        .onTrue(new InstantCommand(() -> shooter.jogSpeed(shooter.k.jogSpeedIncriment)));

    inputs.shiftB6
        .and(inputs.shootAngleJogDn)
        .onTrue(new InstantCommand(() -> shooter.jogSpeed(-shooter.k.jogSpeedIncriment)));
    
    inputs.shiftB6
        .and(inputs.armAngleJogUp)
        .onTrue(new InstantCommand(climber::winchJogLeft));
    
    inputs.shiftB6
        .and(inputs.armAngleJogDn)
        .onTrue(new InstantCommand(climber::winchJogRight));

    //TODO: map here for now
    inputs.SWC.and(inputs.SWBHi.negate().and(inputs.SWBLo.negate())).whileTrue(CmdGather.unGather(this));
    //if in climb mode, ungather will transfer
    //inputs.SWC.and(inputs.SWBHi.or(inputs.SWBLo)).onTrue(CmdTransfer.transferForAmp(this, inputs.SWC));
    //inputs.SWC.and(inputs.SWBHi.or(inputs.SWBLo)).onTrue(CmdClimb.simpleClimb(this));

    //inputs.shootTriggerSWH.and(inputs.SWBHi.or(inputs.SWBLo)).whileTrue(CmdClimb.testClimb(this));
    
  }

  public Command getAutonomousCommand() {
    return autonCommand;
  }
  
  private Command autonCommand = new InstantCommand();
  private String lastSelectedAuton = "";
  private Pose2d autonStartPose = new Pose2d();
  
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
    selectedAuton += getAlliance();
    selectedAuton += startChooser.get().ordinal();
    selectedAuton += waitChooser.get();

    if (/*checkPoseError(autonStartPose, drive.getPose()) || */!selectedAuton.equals(lastSelectedAuton)){
      Locations.recalcForAlliance();
      
      lastSelectedAuton = selectedAuton;

      /*
      switch(startChooser.get()){
        case SOURCE_SIDE:
          drive.resetFieldOdometry(Locations.startLocations[StartLocationType.SOURCE_SIDE.ordinal()]);
          break;
        case SPEAKER_CENTER:
          drive.resetFieldOdometry(Locations.startLocations[StartLocationType.SPEAKER_CENTER.ordinal()]);
          break;
        case AMP_SIDE:
          drive.resetFieldOdometry(Locations.startLocations[StartLocationType.AMP_SIDE.ordinal()]);
          break;
        case APRILTAG:

          break;
        case APRILTAG_0Deg:
          drive.resetFieldOrientedAngle(Locations.startLocations[1].getRotation());
          break;
        default:
          break;
      }
      */

      //autonStartPose = drive.getPose();

      switch(autoChooser.get()){
        //TODO: include totalNotes in the pregen path autos to stop early
        //Probably do this as part of a AutonMonitor running in parallel
        case DENIAL:
          drive.k.dontFlip = false;
          autonCommand = new InstantCommand().beforeStarting(CmdAuton.resetPosition(this, startChooser.get()));
          break;

        case PREGEN:
          drive.k.dontFlip = false;
          autonCommand = ChoreoAuto.getPregen3NoteMid("3NoteMid", this);
          break;

        case SELECTABLE:
          drive.k.dontFlip = true;
          autonCommand = CmdAuton.selectedAuto(this,
                                notePriorityA.get(),
                                notePriorityB.get(),
                                notePriorityC.get(),
                                notePriorityD.get(),
                                notePriorityE.get(),
                                notePriorityF.get(),
                                notePriorityG.get(),
                                notePriorityH.get(),
                                totalNotes.get(),
                                startChooser.get(),
                                waitChooser.get()
          );
          break;

        case TEST:
          drive.k.dontFlip = false;
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
                    new PIDConstants(9.0, 0.0, 0.75), // Rotation PID constants
                    drive.k.maxWheelSpeed, // Max module speed, in m/s
                    drive.k.wheelBR.wheelLocation.getNorm(), // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            drive.k::flipPath,
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
    startChooser = new LoggedDashboardChooser<>("Start Location");
    waitChooser = new LoggedDashboardChooser<>("Start Wait Time");

    autoChooser.addDefaultOption("Do Nothing", AutonType.DO_NOTHING);
      autoChooser.addOption("Pregenerated", AutonType.PREGEN);
      autoChooser.addOption("Denial", AutonType.DENIAL);
      autoChooser.addOption("Selectable", AutonType.SELECTABLE);
      autoChooser.addOption("Test", AutonType.TEST);

    startChooser.addDefaultOption("Center", StartLocationType.SPEAKER_CENTER);
      startChooser.addOption("AmpSide", StartLocationType.AMP_SIDE);
      startChooser.addOption("SourceSide", StartLocationType.SOURCE_SIDE);
      startChooser.addOption("AprilTag", StartLocationType.APRILTAG);
      startChooser.addOption("AprilTag 0 Angle", StartLocationType.APRILTAG_0Deg);

    waitChooser.addDefaultOption("0sec", 0);
      waitChooser.addOption("1sec", 1);
      waitChooser.addOption("2sec", 2);
      waitChooser.addOption("3sec", 3);
      waitChooser.addOption("4sec", 4);
      waitChooser.addOption("5sec", 5);
      waitChooser.addOption("6sec", 6);
      waitChooser.addOption("7sec", 7);
      waitChooser.addOption("8sec", 8);
      waitChooser.addOption("9sec", 9);
      waitChooser.addOption("10sec", 10);
    
    addNoteOrderHelper(notePriorityA);
    addNoteOrderHelper(notePriorityB);
    addNoteOrderHelper(notePriorityC);
    addNoteOrderHelper(notePriorityD);
    addNoteOrderHelper(notePriorityE);
    addNoteOrderHelper(notePriorityF);
    addNoteOrderHelper(notePriorityG);
    addNoteOrderHelper(notePriorityH);
    totalNotes.addDefaultOption("0", 0);
    totalNotes.addOption("1", 1);
    totalNotes.addOption("2", 2);
    totalNotes.addOption("3", 3);
    totalNotes.addOption("4", 4);
    totalNotes.addOption("5", 5);
    totalNotes.addOption("6", 6);
    totalNotes.addOption("7", 7);
    totalNotes.addOption("8", 8);

    ShuffleboardTab autoTab = Shuffleboard.getTab("Auton");
    autoTab.add("Auton Mode", autoChooser.getSendableChooser()).withPosition(0, 0).withSize(2,1);
    autoTab.add("Total Notes", totalNotes.getSendableChooser()).withPosition(2, 0).withSize(2,1);
    autoTab.add("Start Location", startChooser.getSendableChooser()).withPosition(4, 0).withSize(2,1);
    autoTab.add("Note A", notePriorityA.getSendableChooser()).withPosition(1, 1);
    autoTab.add("Note B", notePriorityB.getSendableChooser()).withPosition(2, 1);
    autoTab.add("Note C", notePriorityC.getSendableChooser()).withPosition(3, 1);
    autoTab.add("Note D", notePriorityD.getSendableChooser()).withPosition(4, 1);
    autoTab.add("Note E", notePriorityE.getSendableChooser()).withPosition(5, 1);
    autoTab.add("Note F", notePriorityF.getSendableChooser()).withPosition(2, 2);
    autoTab.add("Note G", notePriorityG.getSendableChooser()).withPosition(3, 2);
    autoTab.add("Note H", notePriorityH.getSendableChooser()).withPosition(4, 2);
  }

  private void addNoteOrderHelper(LoggedDashboardChooser<Integer> c){
    c.addDefaultOption("N/A", 0);
    c.addOption("1st", 1);
    c.addOption("2nd", 2);
    c.addOption("3rd", 3);
    c.addOption("4th", 4);
    c.addOption("5th", 5);
    c.addOption("6th", 6);
    c.addOption("7th", 7);
    c.addOption("8th", 8);
  }

  private boolean checkPoseError(Pose2d one, Pose2d two){
    return one.getTranslation().minus(two.getTranslation()).getNorm() > Units.inchesToMeters(12)
            || Math.abs(MathUtil.angleModulus(one.getRotation().minus(two.getRotation()).getRadians())) > Units.degreesToRadians(20);
  }

  private int getAlliance(){
    if(DriverStation.getAlliance().isPresent()){
      if(DriverStation.getAlliance().get() == Alliance.Blue){
        return 1;
      } else {
        return 2;
      }
    } else {
      return -1;
    }
  }
}
