// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class LockOn extends Command {
  private Vision vision;
  private Drivetrain drivetrain;
  private Joystick controller;
  
  /** Creates a new LockOn. */
  public LockOn(Drivetrain drivetrain, Vision vision, Joystick controller) {
    this.vision = vision;
    this.drivetrain = drivetrain;
    this.controller = controller;
    addRequirements(drivetrain, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  /* curve such that an x value of 0.0
  * results in a y value of 0.0, and
  * as x goes towards infinity, 
  * y approaches 1.0.
  */
	static double curve(double x) {
    if (x <= 0.0)
      return 0.0;
    if (x >= 1.0)
      return 1.0;
    return x;
	}  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double yawTolerance = 0.0;
    
    vision.updateTargetsList();
    
    final int tagID = 1;
    int index = vision.getTagIndex(tagID);

    double translationX = deadband(-controller.getRawAxis(XboxController.Axis.kLeftY.value));
    double translationY = deadband(-controller.getRawAxis(XboxController.Axis.kLeftX.value));
    Translation2d translation = new Translation2d(translationX, translationY).times(Constants.DrivetrainConstants.MAX_SPEED);
    double rotation = -controller.getRawAxis(XboxController.Axis.kRightX.value) * Constants.DrivetrainConstants.maxAngularVelocity;

    // possibly use PID here eventually
    if (index != -1) {
      rotation = 0.0;
      
      double yaw = vision.getYaw(index);
      if (yaw <= -yawTolerance) {
          rotation = 0.3 * Constants.DrivetrainConstants.maxAngularVelocity * curve(-yaw / 23.0);
      } 
      else if (yaw >= yawTolerance) {
          rotation = -0.3 * Constants.DrivetrainConstants.maxAngularVelocity * curve(yaw / 23.0);
      } 
      drivetrain.simpleDrive(
        translation,
        rotation
      );

    } else {
      drivetrain.simpleDrive(
        translation,
        rotation
      );
    };
  }

  public double deadband(double value) {
    if (value >= 0.10) {
      return (value - 0.10) / 0.90;
    } else if (value <= -0.10) {
      return (value + 0.10) / 0.90;
    } else {
      return 0.0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("FINISHED FR");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}