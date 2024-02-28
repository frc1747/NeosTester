package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import java.util.function.BooleanSupplier;

import org.ejml.ops.FConvertArrays;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotShooter extends SubsystemBase {
  private TalonFX hinge;
  private DigitalInput limitSwitch;
  double start ;

  /** Creates a new Shooter. */
  public PivotShooter() {
    hinge = new TalonFX(Constants.ShooterConstants.HINGE);
    limitSwitch = new DigitalInput(Constants.ShooterConstants.LIMIT_SWITCH);
    hinge.setNeutralMode(NeutralMode.Brake);
    this.start =  hinge.getSelectedSensorPosition();
    

    configPID();
  }

  public void configPID() {
    double[] pidf = new double[] {0.4, 0, 0, 0};
    hinge.config_kP(0, pidf[0]);
    hinge.config_kI(0, pidf[1]);
    hinge.config_kD(0, pidf[2]);
    hinge.config_kF(0, pidf[3]);
  }

  public void setHingePower(double power) {
    hinge.set(ControlMode.PercentOutput, power);
  }

  public void dropShooter() {
    hinge.set(ControlMode.Position, Constants.ShooterConstants.STOWED);
  }

  public void alignShooterSpeaker() {
    hinge.set(ControlMode.Position, 10.0);
  }

  public void alignShooterAmp() {
    System.out.println("aligning");
    hinge.set(ControlMode.Position, Constants.ShooterConstants.AMP);
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
  public boolean In_limit(double zero){
    System.out.println((hinge.getSelectedSensorPosition() + "+" + (Constants.ShooterConstants.UP_LIMIT + start)));
    return (hinge.getSelectedSensorPosition() < Constants.ShooterConstants.UP_LIMIT-zero );
  }

  @Override

  public void periodic() {
    boolean reverseLimitClosed = hinge.isRevLimitSwitchClosed() == 1;
    if (reverseLimitClosed) {
      setEncoderPos(0);
    }
    // This method will be called once per scheduler run 
  }
}
