package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class IntakeSubsystem {
    
    TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeMotorID,IntakeConstants.intakeBus);
    public IntakeSubsystem() {
        TalonFXConfiguration intakeConfiguration = new TalonFXConfiguration()
        .withFeedback(new FeedbackConfigs()
        .withSensorToMechanismRatio(12))
        .withMotorOutput(new MotorOutputConfigs());        
    }
    
    public void runIntake(double intakeSP){
        intakeMotor.set(intakeSP);
    }
}
