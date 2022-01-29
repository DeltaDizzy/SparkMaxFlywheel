// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Flywheel;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class RunWheelPID extends CommandBase implements Loggable {
  private final Flywheel wheel;
  private int rpmSetpoint;
  @Config(name = "P")
  double p = 0.001;
  @Config(name = "I")
  double i = 0;
  @Config(name = "D")
  double d = 0;
  @Config(name = "F")
  double f = 0;
  double max = 0;
  double min = -1;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunWheelPID(Flywheel subsystem) {
    wheel = subsystem;
    rpmSetpoint = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("COMMAND STARTED");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wheel.UpdatePID(p, i, d, f, max, min);
    wheel.setSpeedPID(rpmSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    wheel.setSpeedPID(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
