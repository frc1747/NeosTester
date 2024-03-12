// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PivotIntake;

public class FloorPickup extends Command {
  private PivotIntake intakePivot;
  private Intake intake;
  private DigitalInput limitSwitch;

  /* creates a new FloorPickup */
  public FloorPickup(Intake intake, PivotIntake intakePivot) {
    this.intake = intake;
    this.intakePivot = intakePivot;
    this.limitSwitch = new DigitalInput(0);
    addRequirements(intakePivot, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setRollerPower(Constants.IntakeConstants.ROLLER_SPEED);
    intakePivot.setHingePower(Constants.IntakeConstants.PIVOT_OUT_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakePivot.getPosition() > Constants.IntakeConstants.DROPPED) {
      intakePivot.setHingePower(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setRollerPower(0.0);
    
    /*intakePivot.setHingePower(-0.1);
    while (true) {
      if (intakePivot.getPosition() <= 0)
        break;
    }
    intakePivot.setHingePower(0.0);
    */
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    return limitSwitch.get();
    // return (intakePivot.getPosition() > Constants.IntakeConstants.DROPPED);
  }
}
