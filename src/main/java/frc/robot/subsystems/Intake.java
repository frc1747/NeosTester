// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax rollerOne;
  private CANSparkMax rollerTwo;
  private TalonFX hinge;
  private DigitalInput limitSwitch;

  /** Creates a new Intake. */
  public Intake() {
    rollerOne = new CANSparkMax(Constants.IntakeConstants.ROLLER_ONE, MotorType.kBrushless);
    rollerTwo = new CANSparkMax(Constants.IntakeConstants.ROLLER_ONE, MotorType.kBrushless);
    hinge = new TalonFX(Constants.IntakeConstants.HINGE);
    limitSwitch = new DigitalInput(Constants.IntakeConstants.LIMIT_SWITCH);
    configPID();
  }

  public void configPID() {
    double[] pidf = new double[] {0.4, 0, 0, 0};
    hinge.config_kP(0, pidf[0]);
    hinge.config_kI(0, pidf[1]);
    hinge.config_kD(0, pidf[2]);
    hinge.config_kF(0, pidf[3]);
  }

  public void setRollerPower(double power) {
    rollerOne.set(power);
    rollerTwo.set(power);
  }

  public void setHingePower(double power) {
    hinge.set(ControlMode.PercentOutput, power);
  }

  public void liftIntake() {
    hinge.set(ControlMode.Position, Constants.IntakeConstants.STOWED);
  }

  public void dropIntake() {
    hinge.set(ControlMode.Position, Constants.IntakeConstants.DROPPED);
  }

  public void setEncoderPos(double position) {
    hinge.setSelectedSensorPosition(position);
  }

  public double getPosition() {
    return hinge.getSelectedSensorPosition();
  }

  public boolean switchPressed() {
    return !limitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
