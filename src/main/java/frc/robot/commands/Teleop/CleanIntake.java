// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PivotIntake;
import frc.robot.subsystems.PivotShooter;

public class CleanIntake extends Command {
  private final PivotIntake intakePivot;
  private final Intake intake;
  private boolean isReversed;
  private BooleanSupplier isDone;
  
  /** Creates a new DefenseBotShooterPreset. */
  public CleanIntake(PivotIntake intakePivot, Intake intake) {
    this.intakePivot = intakePivot;
    this.intake = intake;
    this.isReversed = false;
    this.isDone = () -> false;
    addRequirements(intakePivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public CleanIntake(PivotIntake intakePivot, Intake intake, BooleanSupplier isDone) {
    this.intakePivot = intakePivot;
    this.intake = intake;
    this.isReversed = false;
    this.isDone = isDone;
    addRequirements(intakePivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (intakePivot.getPosition() < Constants.IntakeConstants.CLEAN) {
      intakePivot.setHingePower(Constants.IntakeConstants.PIVOT_IN_SPEED);
    } else {
      isReversed = true;
      intakePivot.setHingePower(-Constants.IntakeConstants.PIVOT_IN_SPEED);
    }
  }

  // can be improved tolerance with PID
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isReversed) {
      if (intakePivot.getPosition() < Constants.IntakeConstants.CLEAN) {
        intakePivot.setHingePower(0);
        intake.setRollerPower(-Constants.IntakeConstants.ROLLER_SPEED_CLEAN);
        return;
      }
    }
    if (intakePivot.getPosition() > Constants.IntakeConstants.CLEAN) {
      intakePivot.setHingePower(0);
      intake.setRollerPower(-Constants.IntakeConstants.ROLLER_SPEED_CLEAN);
      return;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakePivot.setHingePower(0.0);
    intake.setRollerPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isDone.getAsBoolean();
  }
}
