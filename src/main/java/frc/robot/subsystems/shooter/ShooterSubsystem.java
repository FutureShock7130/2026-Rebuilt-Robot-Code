package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    TalonFX shooterUp = new TalonFX(ShooterConstants.shooterUpID, Constants.canBus);
    TalonFX shooterDown = new TalonFX(ShooterConstants.shooterDownID, Constants.canBus);
    TalonFX shooterAngle = new TalonFX(ShooterConstants.shooterAngleID, Constants.canBus);
    double shooterAngleTarget = 0.0;

    public ShooterSubsystem() {
        TalonFXConfiguration shooterUpConfig = new TalonFXConfiguration();
        shooterUpConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterUpConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterUpConfig.CurrentLimits.StatorCurrentLimit = 120;
        shooterUpConfig.Feedback.RotorToSensorRatio = 1;    
        shooterUpConfig.Feedback.SensorToMechanismRatio = 3;
        var slot0Config = shooterUpConfig.Slot0;
        slot0Config.kS = ShooterConstants.Slot0kS;
        slot0Config.kV = ShooterConstants.Slot0kV;
        slot0Config.kP = ShooterConstants.Slot0kP;
        slot0Config.kI = ShooterConstants.Slot0kI;
        slot0Config.kD = ShooterConstants.Slot0kD;
        var motationConfigUp = shooterUpConfig.MotionMagic;
        motationConfigUp.MotionMagicAcceleration = ShooterConstants.MotionMagicAccelerationUp;
        shooterUp.getConfigurator().apply(shooterUpConfig);

        TalonFXConfiguration shooterDownConfig = new TalonFXConfiguration();
        shooterDownConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterDownConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterDownConfig.CurrentLimits.StatorCurrentLimit = 120;
        shooterDownConfig.Feedback.RotorToSensorRatio = 1;
        shooterDownConfig.Feedback.SensorToMechanismRatio = 3;
        var slot0ConfigDown = shooterDownConfig.Slot0;
        slot0ConfigDown.kS = ShooterConstants.Slot0kS;
        slot0ConfigDown.kV = ShooterConstants.Slot0kV;
        slot0ConfigDown.kP = ShooterConstants.Slot0kP;
        slot0ConfigDown.kI = ShooterConstants.Slot0kI;
        slot0ConfigDown.kD = ShooterConstants.Slot0kD;
        var motationConfigDown = shooterDownConfig.MotionMagic;
        motationConfigDown.MotionMagicAcceleration = ShooterConstants.MotionMagicAccelerationDown;
        shooterDown.getConfigurator().apply(shooterDownConfig);

        TalonFXConfiguration shooterAngleConfig = new TalonFXConfiguration();
        shooterAngleConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterAngleConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterAngleConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterAngleConfig.CurrentLimits.StatorCurrentLimit = 120;
        shooterAngleConfig.Feedback.RotorToSensorRatio = 1;
        shooterAngleConfig.Feedback.SensorToMechanismRatio = 150;
        shooterAngleConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        shooterAngleConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        shooterAngleConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        shooterAngleConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        var slot2Config = shooterAngleConfig.Slot2;
        slot2Config.kS = ShooterConstants.Slot2kS;
        slot2Config.kV = ShooterConstants.Slot2kV;
        slot2Config.kP = ShooterConstants.Slot2kP;
        slot2Config.kI = ShooterConstants.Slot2kI;
        slot2Config.kD = ShooterConstants.Slot2kD;
        var motationConfigAngle = shooterAngleConfig.MotionMagic;
        motationConfigAngle.MotionMagicAcceleration = ShooterConstants.MotionMagicAccelerationAngle;
        shooterAngle.getConfigurator().apply(shooterAngleConfig);
    }

    public double getShooterAnglePos() {
        return (shooterAngle.getPosition().getValueAsDouble()); 
    }

    public void setShooterAngle(double shooterAngleTarget) {
        shooterAngle.set(shooterAngleTarget);
    }

    public void setShooterUpSpeed(double upSpeedRPS) {
        // shooterUp.set(speed);
        shooterUp.setControl(new MotionMagicVelocityVoltage(upSpeedRPS));
    }

    public void setShooterDownSpeed(double downSpeedRPS) {
        shooterDown.setControl(new MotionMagicVelocityVoltage(downSpeedRPS));
    }

    public void stop() {
        shooterUp.set(0);
        shooterDown.set(0);
        shooterAngle.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooterPos", getShooterAnglePos());
    }
}
