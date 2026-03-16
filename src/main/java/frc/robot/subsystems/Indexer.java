package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FSLib.util.PhoenixUtil;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
    private final TalonFX upIndex = new TalonFX(kUpIndexId, Constants.kCanivoreBus);
    private final TalonFX downIndex = new TalonFX(kDownIndexId, Constants.kCanivoreBus);

    private final MotionMagicVelocityVoltage upIndexerRequeset = new MotionMagicVelocityVoltage(0)
        .withEnableFOC(true)
        .withUseTimesync(true)
        .withUpdateFreqHz(0);
    private final MotionMagicVelocityVoltage downIndexerRequeset = new MotionMagicVelocityVoltage(0)
        .withEnableFOC(true)
        .withUseTimesync(true)
        .withUpdateFreqHz(0);

    public Indexer() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive);
        config.CurrentLimits
            .withStatorCurrentLimit(80)
            .withSupplyCurrentLimit(40);
        config.MotionMagic
                    .withMotionMagicAcceleration(80)
                    .withMotionMagicJerk(400);
        PhoenixUtil.assertOk(upIndex, () -> upIndex.getConfigurator().apply(config));

        config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        PhoenixUtil.assertOk(downIndex, () -> downIndex.getConfigurator().apply(config));
    }

    public void set(double upSpeed, double downSpeed) {
        upIndex.set(upSpeed);
        downIndex.set(downSpeed);
        // upIndex.setControl(upIndexerRequeset.withVelocity(upRPS));
        // downIndex.setControl(downIndexerRequeset.withVelocity(downRPS));
    }

    public void setVolt(double upVolts, double downVolts) {
        upIndex.setVoltage(upVolts);
        downIndex.setVoltage(downVolts);
    }

    public void stopAll() {
        upIndex.set(0);
        downIndex.set(0);
    }

    @Override
    public void periodic() {
    }
}
