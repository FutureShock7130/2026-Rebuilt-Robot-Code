package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ClimberConstants.*;

import java.util.function.DoubleConsumer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private final TalonFX leftClimber = new TalonFX(kLeftClimberId, Constants.kCanivoreBus);

    private final DynamicMotionMagicVoltage request = new DynamicMotionMagicVoltage(0, 3, 15)
        .withEnableFOC(true);

    private DoubleConsumer telemetry;

    private final DCMotorSim m_motorSimModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 0.001, kSensorToMechanismRatio
        ),
        DCMotor.getKrakenX60Foc(1)
    );

    public Climber() {
        TalonFXConfiguration config = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(120)
                    .withSupplyCurrentLimit(70)
                    .withSupplyCurrentLowerLimit(40)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(kSensorToMechanismRatio)
            )
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKS(0.24).withKV(0.5).withKA(0)
                    .withKP(0.2).withKI(0).withKD(0)
            )
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(kMaxHeightMeters)
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(kMinHeightMeters)
            );

        leftClimber.getConfigurator().apply(config);

        leftClimber.setPosition(kMinHeightMeters / kClimbHeightMeters);

        var talonFXSim = leftClimber.getSimState();
        talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    private Command move(double targetHeightMeters) {
        return this.runOnce(() -> {
            leftClimber.setControl(request.withPosition(targetHeightMeters / kMechanismToHeighRatio));
        });
    }

    public Command preClimb() {
        return move(kPreClimbHeightMeters);
    }

    public Command climb() {
        return move(kClimbHeightMeters);
    }

    public Command retract() {
        return move(0);
    }

    public Command extendThenClimb() {
        return this.startEnd(
            () -> {
                leftClimber.setControl(request.withPosition(kPreClimbHeightMeters));
            },
            () -> {
                leftClimber.setControl(request.withPosition(kClimbHeightMeters));
            }
        );
    }

    public boolean atTargetHeight() {
        return Math.abs(leftClimber.getClosedLoopError().getValueAsDouble()) < 0.05;
    }

    public void registerTelemetry(DoubleConsumer telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        if (telemetry != null) {
            telemetry.accept(leftClimber.getPosition().getValueAsDouble() * kMechanismToHeighRatio);
        }
    }

    @Override
    public void simulationPeriodic() {
        var talonFXSim = leftClimber.getSimState();
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
        m_motorSimModel.update(0.020);

        talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(kSensorToMechanismRatio));
        talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(kSensorToMechanismRatio));
    }
}
