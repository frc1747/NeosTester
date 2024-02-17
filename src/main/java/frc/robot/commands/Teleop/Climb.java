// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class Climb extends Command {
  Climber climber;
  double speed;
  BooleanSupplier rightTrigger;
  BooleanSupplier leftTrigger;
  BooleanSupplier rightBumper;
  BooleanSupplier leftBumper;
 
  
  /** Creates a new Climb. */
  public Climb(Climber climber, double speed,
  BooleanSupplier rightTrigger, BooleanSupplier leftTrigger,
  BooleanSupplier rightBumper, BooleanSupplier leftBumper) {
    
    this.climber = climber;
    this.speed = speed;

    this.rightTrigger = rightTrigger;
    this.leftTrigger = leftTrigger;
    this.rightBumper = rightBumper;
    this.leftBumper = leftBumper;

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (leftBumper.getAsBoolean())
      climber.setLeftPower(speed);
    else if (leftTrigger.getAsBoolean())
      climber.setLeftPower(-speed);
    else 
      climber.setLeftPower(0);

    if (rightBumper.getAsBoolean())
      climber.setRightPower(speed);
    else if (rightTrigger.getAsBoolean())
      climber.setRightPower(-speed);
    else 
      climber.setRightPower(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setLeftPower(0.0);
    climber.setRightPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
