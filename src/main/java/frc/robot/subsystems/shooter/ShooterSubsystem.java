package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    TalonFX shooterUp = new TalonFX(ShooterConstants.shooterUpID, Constants.canBus);
    TalonFX shooterDown = new TalonFX(ShooterConstants.shooterDownID, Constants.canBus);
    TalonFX shooterAngle = new TalonFX(ShooterConstants.shooterAngleID, Constants.canBus);
    PIDController shooterAnglePID = new PIDController(
            ShooterConstants.anglekP,
            ShooterConstants.anglekI,
            ShooterConstants.anglekD);
    double shooterAngleTarget = 0.0;

    public ShooterSubsystem() {
        var shooterUpConfig = new TalonFXConfiguration();
        var slot0Config = shooterUpConfig.Slot0;
        slot0Config.kS = ShooterConstants.Slot0kS;
        slot0Config.kV = ShooterConstants.Slot0kV;
        slot0Config.kP = ShooterConstants.Slot0kP;
        slot0Config.kI = ShooterConstants.Slot0kI;
        slot0Config.kD = ShooterConstants.Slot0kD;
        var motationConfigUp = shooterUpConfig.MotionMagic;
        motationConfigUp.MotionMagicAcceleration = ShooterConstants.MotionMagicAccelerationUp;
        shooterUp.getConfigurator().apply(shooterUpConfig);

        var shooterDownConfig = new TalonFXConfiguration();
        var slot1Config = shooterDownConfig.Slot1;
        slot1Config.kS = ShooterConstants.Slot1kS;
        slot1Config.kV = ShooterConstants.Slot1kV;
        slot1Config.kP = ShooterConstants.Slot1kP;
        slot1Config.kI = ShooterConstants.Slot1kI;
        slot1Config.kD = ShooterConstants.Slot1kD;
        var motationConfigDown = shooterDownConfig.MotionMagic;
        motationConfigDown.MotionMagicAcceleration = ShooterConstants.MotionMagicAccelerationDown;
        shooterDown.getConfigurator().apply(shooterDownConfig);

        TalonFXConfiguration shooterAngleConfig = new TalonFXConfiguration();
        shooterAngleConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterAngleConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterAngleConfig.CurrentLimits.SupplyCurrentLimit = 0;
        shooterAngle.getConfigurator().apply(shooterAngleConfig);
    }

    public double getShooterAnglePos() {
        return (shooterAngle.getPosition().getValueAsDouble()) / 150 * 360; // degrees???
    }

    public void setShooterTarget(double angle) {
        shooterAngleTarget = angle;
    }

    public void setShooterAngle() {
        double setShooterAngle = (shooterAnglePID.calculate(getShooterAnglePos(), shooterAngleTarget));
        shooterAngle.set(setShooterAngle);
    }

    public void setShooterUpSpeed(double upSpeedRPM) {
        // shooterUp.set(speed);
        double upSpeedRPS = upSpeedRPM / 60;
        shooterUp.setControl(new MotionMagicVelocityVoltage(upSpeedRPS));
    }

    public void setShooterDownSpeed(double downSpeedRPM) {
        double downSpeedRPS = downSpeedRPM / 60;
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
