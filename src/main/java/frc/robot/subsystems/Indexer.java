package frc.robot.subsystems;

import static frc.robot.Constants.IndexerContants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FSLib.util.PhoenixUtil;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
    private final TalonFX upIndex = new TalonFX(kUpIndexId, Constants.kCanivoreBus);
    private final TalonFX downIndex = new TalonFX(kDownIndexId, Constants.kCanivoreBus);

    public Indexer() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);
        config.CurrentLimits
            .withStatorCurrentLimit(120)
            .withSupplyCurrentLimit(70);
        PhoenixUtil.assertOk(upIndex, () -> upIndex.getConfigurator().apply(config));
        PhoenixUtil.assertOk(downIndex, () -> downIndex.getConfigurator().apply(config));
    }

    public void set(double upSpeed, double downSpeed) {
        upIndex.set(upSpeed);
        downIndex.set(downSpeed);
    }

    public void stopAll() {
        upIndex.set(0);
        downIndex.set(0);
    }

    @Override
    public void periodic() {
    }
}
