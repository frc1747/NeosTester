package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs() {
        swerveCanCoderConfig = new CANCoderConfiguration();
        
        // Swerve CANCoder Configuration
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.DrivetrainConstants.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}