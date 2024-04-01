// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PivotShooter;

public class ShooterPivotPreset extends Command {
  private final PivotShooter shooterPivot;
  private final double encoderPosition;
  private boolean isReversed;
  
  /** Creates a new ShooterPivotPreset. */
  public ShooterPivotPreset(PivotShooter shooterPivot, double encoderPosition) {
    this.shooterPivot = shooterPivot;
    this.encoderPosition = encoderPosition;
    isReversed = false;
    addRequirements(shooterPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isReversed = false;
    if (shooterPivot.getPosition() < encoderPosition) {
      shooterPivot.setHingePower(Constants.ShooterConstants.HINGE_SPEED);
    } else {
      isReversed = true;
      shooterPivot.setHingePower(-Constants.ShooterConstants.HINGE_SPEED);
    }
  }

  // can be improved tolerance with PID
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterPivot.setHingePower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isReversed) {
      return (shooterPivot.getPosition() < encoderPosition);
    }
    return (shooterPivot.getPosition() > encoderPosition);
  }
}
