package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.ejml.ops.FConvertArrays;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  private TalonFX shooterFeed;
  
  /** Creates a new Shooter. */
  public Feeder() {
    shooterFeed = new TalonFX(Constants.FeederConstants.BACK);
    shooterFeed.setNeutralMode(NeutralMode.Brake);
  }

  public void setShooterFeedPower(double power) {
    shooterFeed.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
