package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.*;

import java.util.function.BiConsumer;
import java.util.function.DoubleConsumer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FSLib.util.PhoenixUtil;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private final TalonFX angleMotor = new TalonFX(kAngleId, Constants.kCanivoreBus);
    private final TalonFX upShooter = new TalonFX(kUpShooterId, Constants.kCanivoreBus);
    private final TalonFX downShooter = new TalonFX(kDownShooterId, Constants.kCanivoreBus);
    private final TalonFX downShooter2 = new TalonFX(kDownShooter2Id, Constants.kCanivoreBus);

    private final DynamicMotionMagicVoltage angleRequest = new DynamicMotionMagicVoltage(0, 0.5, 2.5).withEnableFOC(true);

    private final MotionMagicVelocityVoltage upShooterRequeset = new MotionMagicVelocityVoltage(0)
        .withEnableFOC(true)
        .withUseTimesync(true)
        .withUpdateFreqHz(0);
    private final MotionMagicVelocityVoltage downShooterRequest = new MotionMagicVelocityVoltage(0)
        .withEnableFOC(true)
        .withUseTimesync(true)
        .withUpdateFreqHz(0);

    private DoubleConsumer telemetry;
    private BiConsumer<Double, Double> speedsTelemetry;

    private final DCMotorSim upShooterSimModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 0.05, kSensorToUpShooterRatio
        ),
        DCMotor.getKrakenX60Foc(1)
    );

    private final DCMotorSim angleSimModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 0.01, kSensorToAngleRatio
        ),
        DCMotor.getKrakenX60Foc(1)
    );

    public Shooter() {
        TalonFXConfiguration angleConfig = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(40)
                    .withSupplyCurrentLimit(20)
                    .withSupplyCurrentLowerLimit(20)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(kSensorToAngleRatio)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(0.5)
                    .withMotionMagicAcceleration(2.5)
                    .withMotionMagicJerk(25)
            )
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKG(0.3).withKS(0.0).withKV(15.0).withKA(0.05)
                    .withKP(100.0).withKI(0).withKD(0)
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withGravityArmPositionOffset(0.02)
            )
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(kAngleMax)
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(kAngleMin)
            );
        
        PhoenixUtil.assertOk(angleMotor, () -> angleMotor.getConfigurator().apply(angleConfig));

        TalonFXConfiguration upShooterConfig = new TalonFXConfiguration()
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(kSensorToUpShooterRatio)
            )
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(80)
                    .withMotionMagicJerk(400)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKS(0.27006).withKV(0.37283).withKA(0.0049632)
                    .withKP(0.068434).withKI(0.05).withKD(0.0)
            );
        
        PhoenixUtil.assertOk(upShooter, () -> upShooter.getConfigurator().apply(upShooterConfig));

        TalonFXConfiguration downShooterConfig = new TalonFXConfiguration()
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(kSensorToDownShooterRatio)
            )
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(80)
                    .withMotionMagicJerk(400)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKS(0.23895).withKV(0.06425).withKA(0.015212)
                    .withKP(0.081285).withKI(0.004).withKD(0.0)
            );
        
        PhoenixUtil.assertOk(downShooter, () -> downShooter.getConfigurator().apply(downShooterConfig));
        PhoenixUtil.assertOk(downShooter2, () -> downShooter2.getConfigurator().apply(downShooterConfig));

        angleMotor.setPosition(0.0);

        var upShooterSim = upShooter.getSimState();
        upShooterSim.Orientation = ChassisReference.CounterClockwise_Positive;
        upShooterSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);

        var angleSim = angleMotor.getSimState();
        angleSim.Orientation = ChassisReference.CounterClockwise_Positive;
        angleSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    public void setSpeed(double upShooterRPS, double downShooterRPS) {
        upShooter.setControl(upShooterRequeset.withVelocity(upShooterRPS));
        downShooter.setControl(downShooterRequest.withVelocity(downShooterRPS));
        downShooter2.setControl(downShooterRequest.withVelocity(downShooterRPS));
    }

    public void setAngle(double angleRotations) {
        angleMotor.setControl(angleRequest.withPosition(angleRotations));
    }

    public boolean atTarget() {
        return atTargetSpeed() && atTargetAngle();
    }

    private boolean atTargetSpeed() {
        return upShooter.getMotionMagicAtTarget().getValue() && downShooter.getMotionMagicAtTarget().getValue();
    }

    private boolean atTargetAngle() {
        return angleMotor.getMotionMagicAtTarget().getValue();
    }

    public void registerTelemetry(DoubleConsumer telemetry, BiConsumer<Double, Double> speedsTelemetry) {
        this.telemetry = telemetry;
        this.speedsTelemetry = speedsTelemetry;
    }

    @Override
    public void periodic() {
        if (telemetry != null) {
            telemetry.accept(angleMotor.getPosition().getValueAsDouble());
        }
        if (speedsTelemetry != null) {
            speedsTelemetry.accept(upShooter.getVelocity().getValueAsDouble(), downShooter.getVelocity().getValueAsDouble());
        }
    }

    @Override
    public void simulationPeriodic() {
        var upShooterSim = upShooter.getSimState();
        upShooterSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        var upShooterVoltage = upShooterSim.getMotorVoltageMeasure();

        upShooterSimModel.setInputVoltage(upShooterVoltage.in(Volts));
        upShooterSimModel.update(0.020);

        upShooterSim.setRawRotorPosition(upShooterSimModel.getAngularPosition().times(kSensorToUpShooterRatio));
        upShooterSim.setRotorVelocity(upShooterSimModel.getAngularVelocity().times(kSensorToDownShooterRatio));

        var angleSim = angleMotor.getSimState();
        angleSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        var angleVoltage = angleSim.getMotorVoltageMeasure();

        angleSimModel.setInputVoltage(angleVoltage.in(Volts));
        angleSimModel.update(0.020);

        angleSim.setRawRotorPosition(angleSimModel.getAngularPosition().times(kSensorToAngleRatio));
        angleSim.setRotorVelocity(angleSimModel.getAngularVelocity().times(kSensorToAngleRatio));
    }
}