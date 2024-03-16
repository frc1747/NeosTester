// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autoscommands;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PivotIntake;

public class IntakeAutos extends Command {
  /** Creates a new Intake. */
  Intake pIntake;
  PivotIntake pivot; 
  String type;
  public IntakeAutos(Intake pIntake, PivotIntake pivot ,String type  ) {

    this.pivot= pivot;
    this.pIntake = pIntake;
    this.type = type;



    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (type.equals("stow" )) {
      pIntake.setRollerPower(0);
      pivot.liftIntake(); 
    
    }
    else if(type.equals("floor")){
      pIntake.setRollerPower(.4);
      pivot.dropIntake();
      try {
        TimeUnit.SECONDS.sleep(1);
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
      pivot.liftIntake(); 
      


    }


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.setHingePower(0);
    pIntake.setRollerPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
