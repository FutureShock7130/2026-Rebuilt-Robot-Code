package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ClimberConstants.*;

import java.util.function.DoubleConsumer;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANrange;
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
    private final TalonFX leftClimber = new TalonFX(kMotorId, Constants.kCanivoreBus);
    private final CANrange canrange = new CANrange(kCANRangeId, Constants.kCanivoreBus);

    private final DynamicMotionMagicVoltage request = new DynamicMotionMagicVoltage(0, 50, 250)
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
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKS(0.0).withKV(8).withKA(0.25)
                    .withKP(20.0).withKI(0).withKD(0)
            )
            .withSlot1(
                new Slot1Configs()
                    .withKS(0.0).withKV(8).withKA(0.25)
                    .withKP(20.0).withKI(0).withKD(0)
            )
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(kMaxHeightMeters / kMechanismToHeighRatio)
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(kMinHeightMeters / kMechanismToHeighRatio)
            );

        leftClimber.getConfigurator().apply(config);

        leftClimber.setPosition(0);

        CANrangeConfiguration canrangeConfig = new CANrangeConfiguration()
            .withProximityParams(
                new ProximityParamsConfigs()
            );

        canrange.getConfigurator().apply(canrangeConfig);

        var talonFXSim = leftClimber.getSimState();
        talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    public Command toPreClimbPos() {
        return this.run(
            () -> {
                leftClimber.setControl(request.withPosition(kPreClimbHeightMeters / kMechanismToHeighRatio).withSlot(0));
            }
        ).withName("ClimberToPreClimbPos");
    }

    public Command toClimbPos() {
        return this.run(
            () -> {
                leftClimber.setControl(request.withPosition(kClimbHeightMeters / kMechanismToHeighRatio).withSlot(1));
            }
        ).withName("ClimberToClimbPos");
    }

    public Command toDefaultPose() {
        return this.run(
            () -> {
                leftClimber.setControl(request.withPosition(0).withSlot(0));
            }
        ).withName("ClimberToDefaultPos");
    }

    public boolean atTargetHeight() {
        return Math.abs(leftClimber.getClosedLoopError().getValueAsDouble()) < 0.05;
    }

    public boolean isAlignedToTower() {
        return canrange.getIsDetected().getValue();
    }

    public double getDistance() {
        return canrange.getDistance().getValueAsDouble();
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
