package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Climber extends SubsystemBase {
  private TalonFX winchLeft, winchRight;
  

  //Enum for climber states
  public enum climberPistonPosition{
    FORWARDS, VERTICAL
  }

  /**
   * creates a climber object to control the climber on the robot
   * @author da bois
   */
  public Climber() {
  
    winchLeft = new TalonFX(Constants.ClimberConstants.LEFT);
    winchRight = new TalonFX(Constants.ClimberConstants.RIGHT);

    winchLeft.setNeutralMode(NeutralMode.Brake);
    winchRight.setNeutralMode(NeutralMode.Brake);
    
    // winchLeft.setInverted(false);
    // winchLeft.setSensorPhase(false);

    // winchRight.setInverted(true);
    // winchRight.setSensorPhase(true);

    setPidsLeft(new double[] {0.4, 0, 0, 0});
    setPidsRight(new double[] {0.4, 0, 0, 0});

    winchLeft.configPeakOutputForward(0.75); 
    winchRight.configPeakOutputForward(0.75);
    winchLeft.configPeakOutputReverse(-0.75);
    winchRight.configPeakOutputReverse(-0.75);

    
		winchLeft.configMotionCruiseVelocity(9000);
		winchLeft.configMotionAcceleration(3000);
    winchLeft.configMotionSCurveStrength(0);

		winchRight.configMotionCruiseVelocity(9000);
		winchRight.configMotionAcceleration(3000);
    winchRight.configMotionSCurveStrength(0);

    winchLeft.setInverted(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber/Right encoder", getRightDistance());
    SmartDashboard.putNumber("Climber/Left encoder", getLeftDistance());
    SmartDashboard.putNumber("Climber/Right Current", getRightCurrent());
    SmartDashboard.putNumber("Climber/Left Current", getLeftCurrent());
  }

  public void cruiseVelocity(int encoderPerSecond) {
    winchLeft.configMotionCruiseVelocity(encoderPerSecond / 10);
    winchRight.configMotionCruiseVelocity(encoderPerSecond / 10);
  }

  public void acceleration(int encoderPerSecondSqr) {
    winchLeft.configMotionAcceleration(encoderPerSecondSqr / 10);
    winchRight.configMotionAcceleration(encoderPerSecondSqr / 10);
  }

  public void setPidsLeft(double[] pidf) {
    winchLeft.config_kP(0, pidf[0]);
    winchLeft.config_kI(0, pidf[1]);
    winchLeft.config_kD(0, pidf[2]);
    winchLeft.config_kF(0, pidf[3]);
  }

  public void setPidsRight(double[] pidf) {
    winchRight.config_kP(0, pidf[0]);
    winchRight.config_kI(0, pidf[1]);
    winchRight.config_kD(0, pidf[2]);
    winchRight.config_kF(0, pidf[3]);
  }

  // sets the encoder to 0
  public void zeroRight() {
    winchRight.setSelectedSensorPosition(0);
  }

  // sets the encoder to 0
  public void zeroLeft() {
    winchLeft.setSelectedSensorPosition(0);
  }

  public double getLeftDistance() {
    return winchLeft.getSelectedSensorPosition();
  }

  public double getLeftSpeed() {
    return winchLeft.getSelectedSensorVelocity();
  }

  public double getLeftCurrent() {
    return winchLeft.getSupplyCurrent();
  }



  public double getRightDistance() {
    return winchRight.getSelectedSensorPosition();
  }

  public double getRightSpeed() {
    return winchRight.getSelectedSensorVelocity();
  }
  
  public double getRightCurrent() {
    return winchRight.getSupplyCurrent();
  }
  
  /**
   * @param power The percentage of power being sent to the left winch motor
   * @author Foz
   */
  public void setLeftPower(double power) {
      winchLeft.set(ControlMode.PercentOutput, power);
  }
  
  /**
   * @param power The percentage of power being sent to the right winch motor
   * @author Foz
   */
  public void setRightPower(double power) {
    winchRight.set(ControlMode.PercentOutput, power);
  }

  /**
   * @param setPoint The positional setpoint for the left winch
   * @author Foz
   */
  public void SetSetpointLeft(double setPoint){
    winchLeft.set(ControlMode.Position, setPoint);
  }

  /**
   * @param setPoint the positional setpoint for the right winch
   * @author Foz
   */
  public void SetSetpointRight(double setPoint){
    winchRight.set(ControlMode.Position, setPoint);
  }


  public void setMotionMagicSetpoint(double setPoint) {
    winchLeft.set(ControlMode.MotionMagic, setPoint);
    winchRight.set(ControlMode.MotionMagic, setPoint);
  }

  /**
   * Extends the large pistons
   * @author Cobob
   */
 
  public TalonFX[] gibMotors() {
    return new TalonFX[] {winchLeft, winchRight};
  }
}
