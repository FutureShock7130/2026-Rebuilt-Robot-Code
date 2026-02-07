package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FSLib.util.PhoenixUtil;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final TalonFX motor = new TalonFX(kMotorId, Constants.kCanivoreBus);

    public Intake() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        config.CurrentLimits
            .withStatorCurrentLimit(40)
            .withSupplyCurrentLimit(20)
            .withSupplyCurrentLowerLimit(20)
            .withSupplyCurrentLowerTime(0.0);

        PhoenixUtil.assertOk(motor, () -> motor.getConfigurator().apply(config));
    }

    public Command set(double speed) {
        return Commands.startEnd(() -> motor.set(speed), () -> motor.set(0), this);
    }

    @Override
    public void periodic() {
    }
}
