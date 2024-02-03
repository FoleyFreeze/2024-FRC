// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.cals.DriveCals;
import frc.robot.cals.GatherCals;
import frc.robot.cals.InputsCals;
import frc.robot.cals.VisionCals;
import frc.robot.commands.drive.CmdDrive;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gather.Gather;
import frc.robot.subsystems.inputs.Inputs;
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

  public RobotContainer() {
    inputs = new Inputs(this, new InputsCals());
    drive = new Drive(this, new DriveCals());
    gather = new Gather(this, new GatherCals());
    vision = new Vision(this, new VisionCals());
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(new CmdDrive(this).ignoringDisable(true));

    inputs.resetSwerveZeros.onTrue(new InstantCommand(drive::learnSwerveOffsets).ignoringDisable(true));

    inputs.gatherTrigger.whileTrue(new RunCommand( () -> gather.setMotorPower(inputs.flysky.getRawAxis(7)/2.0 + 0.5), gather).finallyDo(() -> gather.setMotorPower(0)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}
