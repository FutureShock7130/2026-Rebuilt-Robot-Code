package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    CANBus indexerUpCanBus = new CANBus("GTX7130");
    TalonFX indexerUp = new TalonFX(53, indexerUpCanBus);
    CANBus indexerDownCanBus = new CANBus("GTX7130");
    TalonFX indexerDown = new TalonFX(54, indexerDownCanBus);
    public IndexerSubsystem() {
    }
    public void setIndexerUpSpeed(double speedUp){
        indexerUp.set(speedUp);        
    }
    public void setIndexerDownSpeed(double speedDown){
        indexerUp.set(speedDown);        
    }
    public void stop() {
        indexerUp.set(0);
        indexerDown.set(0);
    }
}
