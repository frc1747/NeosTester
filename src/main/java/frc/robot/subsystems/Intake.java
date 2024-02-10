// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax rollerOne;
  private CANSparkMax rollerTwo;
  private TalonFX hinge;

  /** Creates a new Intake. */
  public Intake() {
    rollerOne = new CANSparkMax(Constants.IntakeConstants.ROLLER_ONE, MotorType.kBrushless);
    rollerTwo = new CANSparkMax(Constants.IntakeConstants.ROLLER_ONE, MotorType.kBrushless);
    hinge = new TalonFX(Constants.IntakeConstants.HINGE);
  }

  public void setRollerPower(double power) {
    rollerOne.set(power);
    rollerTwo.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
