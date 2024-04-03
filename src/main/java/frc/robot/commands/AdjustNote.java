// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;

public class AdjustNote extends Command {
  Feeder feeder;
  long startTime;
  
  /** Creates a new AdjustNote. */
  public AdjustNote(Feeder feeder) {
    this.feeder = feeder;
    this.startTime = System.currentTimeMillis();
    addRequirements(feeder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.startTime = System.currentTimeMillis();
    feeder.setShooterFeedPower(-Constants.FeederConstants.TRANSITION_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.setShooterFeedPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - startTime) >= Constants.FeederConstants.ADJUST_NOTE_MILLIS;
  }
}
