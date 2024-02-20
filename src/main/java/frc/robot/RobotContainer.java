// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.Teleop.Climb;
import frc.robot.commands.Teleop.TeleopSwerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.StadiaController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static CTREConfigs ctreConfigs = new CTREConfigs();

  // subsystems
  public final Drivetrain drivetrain = new Drivetrain();
  public final Climber climber = new Climber();


  // Controllers
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

  // Drive Controls
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // Driver Buttons
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  // BooleanSuppliers
  private final BooleanSupplier rightTrigger = () -> XboxController.Axis.kRightTrigger.value > Short.MAX_VALUE - 10;
  private final BooleanSupplier leftTrigger = () -> XboxController.Axis.kLeftTrigger.value > Short.MAX_VALUE - 10;
  private final BooleanSupplier rightBumper = () -> XboxController.Button.kRightBumper.value == 1;
  private final BooleanSupplier leftBumper = () -> XboxController.Button.kLeftBumper.value == 1;
  
  // climber Controls speeds

  double climberspeed = -.2;
 
  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    drivetrain.setDefaultCommand(
      new TeleopSwerve(
        drivetrain, 
        () -> -driver.getRawAxis(translationAxis), 
        () -> -driver.getRawAxis(strafeAxis), 
        () -> -driver.getRawAxis(rotationAxis), 
        () -> robotCentric.getAsBoolean()
      )
    );

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));
    // used to swicth the climber going up 0or down
  
    new JoystickButton(operator, XboxController.Button.kRightBumper.value)
      .whileTrue(new Climb(climber, 0.20, rightTrigger, leftTrigger, rightBumper, leftBumper));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null; // Autos.exampleAuto(m_exampleSubsystem);
  }
}
