package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
    TalonFX indexerUp = new TalonFX(IndexerConstants.indexerUpID, Constants.canBus);
    TalonFX indexerDown = new TalonFX(IndexerConstants.indexerDownID, Constants.canBus);

    public IndexerSubsystem() {
        TalonFXConfiguration indexerUpConfig = new TalonFXConfiguration();
        indexerUpConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        indexerUpConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexerUpConfig.CurrentLimits.SupplyCurrentLimit = 0;
        indexerUp.getConfigurator().apply(indexerUpConfig);

        TalonFXConfiguration indexerDownConfig = new TalonFXConfiguration();
        indexerDownConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        indexerDownConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexerDownConfig.CurrentLimits.SupplyCurrentLimit = 0;
        indexerDown.getConfigurator().apply(indexerDownConfig);
    }

    public void setIndexerUpSpeed(double speedUp) {
        indexerUp.set(speedUp);
    }

    public void setIndexerDownSpeed(double speedDown) {
        indexerDown.set(speedDown);
    }

    public void stop() {
        indexerUp.set(0);
        indexerDown.set(0);
    }

    public Command run(double speedUp, double speedDown) {
        return this.runEnd(
            () -> {
                setIndexerUpSpeed(speedUp);
                setIndexerDownSpeed(speedDown);
            },
            () -> stop()
        );
    }
}
