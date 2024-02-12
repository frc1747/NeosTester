package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private TalonFX front;
  private TalonFX back;
  private TalonFX hinge;

  /** Creates a new Shooter. */
  public Shooter() {
    front = new TalonFX(Constants.ShooterConstants.FRONT);
    back = new TalonFX(Constants.ShooterConstants.BACK);
    hinge = new TalonFX(Constants.ShooterConstants.HINGE);
    front.setNeutralMode(NeutralModeValue.Brake);
    back.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setShooterPower(double power1, double power2) {
    front.set(power1);
    back.set(power2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
