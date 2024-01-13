// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  public SwerveModule[] swerveModules;
  public Drivetrain() {

    swerveModules = new SwerveModule[] {
      new SwerveModule(0, null),
      new SwerveModule(0, null),
      new SwerveModule(0, null),
      new SwerveModule(0, null)
    };
  }

  @Override
  public void periodic() {
  }












  public class SwerveModule {
    // This might go in its own class, but am leaving it here for now

    public int moduleNumber;

    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
    private CANCoder angleEncoder;

    private SparkPIDController driveController;  // Deprecated from SparkMaxPIDController
    private SparkPIDController angleController;

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.DrivetrainConstants.DRIVE_KS, Constants.DrivetrainConstants.DRIVE_KV, Constants.DrivetrainConstants.DRIVE_KA);


    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {  // TODO: Pass in constants
      this.moduleNumber = moduleNumber;
      this.angleOffset = moduleConstants.angleOffset;

      // CANCoder
      angleEncoder = new CANCoder(moduleConstants.cancoderID);  // Fix this deprecation later
      // configAngleEncoder();

      // Angle Motor
      angleMotor = new CANSparkMax(0, MotorType.kBrushless);
      integratedAngleEncoder = angleMotor.getEncoder();
      angleController = angleMotor.getPIDController();
      // configAngleMotor();

      // Drive Motor
      driveMotor = new CANSparkMax(0, MotorType.kBrushless);
      driveEncoder = driveMotor.getEncoder();
      driveController = driveMotor.getPIDController();
      // configDriveMotor();

      lastAngle = getState().angle;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
      if (isOpenLoop) {
        double percentOutput = desiredState.speedMetersPerSecond / Constants.DrivetrainConstants.MAX_SPEED;
        driveMotor.set(percentOutput);
      } else {
        driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond)
        );
      }
    }

    public void setAngle(SwerveModuleState desiredState) {
      // Prevent roating module if speed is less than 1%
      Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.DrivetrainConstants.MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle;

      angleController.setReference(angle.getDegrees(), ControlType.kPosition);
      lastAngle = angle;
    }    

    public Rotation2d getAngle() {
      return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public Rotation2d getCancoder() {
      return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState() {
      return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition() {
      return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }


    public class SwerveModuleConstants {
      // This is a wrapper class for the constants defined in Constants.Drivetrain

      public final int driveMotorID;
      public final int angleMotorID;
      public final int cancoderID;
      public final Rotation2d angleOffset;

      public SwerveModuleConstants (int driveMotorID, int angleMotorID, int cancoderID, Rotation2d angleOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = cancoderID;
        this.angleOffset = angleOffset;
      }
    }
  }
}
