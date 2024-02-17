// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class BringIn extends Command {
  private Shooter shooter;
  private Intake intake;
  private boolean shooterDone = false;
  private boolean intakeDone = false;

  /** Creates a new BringIn. */
  public BringIn(Shooter shooter, Intake intake) {
    this.shooter = shooter;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!shooterDone) {
      shooter.setHingePower(-0.05);
      if (shooter.switchPressed()) {
        shooter.setHingePower(0.0);
        shooter.setEncoderPos(Constants.ShooterConstants.DOWN_LIMIT);
        shooterDone = true;
      }
    }
    if (!intakeDone) {
      intake.setHingePower(-0.05);
      if (intake.switchPressed()) {
        intake.setHingePower(0.0);
        intake.setEncoderPos(Constants.IntakeConstants.UP_LIMIT);
        intakeDone = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setHingePower(0.0);
    intake.setHingePower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterDone && intakeDone;
  }
}
