package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterContants.*;

import java.util.function.DoubleConsumer;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.FSLib.util.PhoenixUtil;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private final TalonFX angleMotor = new TalonFX(kAngleId, Constants.kCanivoreBus);
    private final TalonFX upShooter = new TalonFX(kUpShooterId, Constants.kCanivoreBus);
    private final TalonFX downShooter = new TalonFX(kDownShooterId, Constants.kCanivoreBus);

    private final DynamicMotionMagicVoltage angleRequest = new DynamicMotionMagicVoltage(0, 0.5, 2.5).withEnableFOC(true);

    private final MotionMagicVelocityVoltage upShooterRequeset = new MotionMagicVelocityVoltage(0)
        .withEnableFOC(true);
        // .withUseTimesync(true)
        // .withUpdateFreqHz(0);
    private final MotionMagicVelocityVoltage downShooterRequest = new MotionMagicVelocityVoltage(0)
        .withEnableFOC(true);
        // .withUseTimesync(true)
        // .withUpdateFreqHz(0);

    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    
    private final SysIdRoutine upShooterRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> SignalLogger.writeString("upShooterState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> upShooter.setControl(voltageRequest.withOutput(volts.in(Volts))),
                null,
                this
            )
        );

    private final SysIdRoutine downShooterRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> SignalLogger.writeString("downShooterState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> downShooter.setControl(voltageRequest.withOutput(volts.in(Volts))),
                null,
                this
            )
    );
    
    private SysIdRoutine sysIdRoutineToApply = downShooterRoutine;

    private DoubleConsumer telemetry;

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
                    .withInverted(InvertedValue.Clockwise_Positive)
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
                    .withMotionMagicAcceleration(50)
                    .withMotionMagicJerk(500)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKS(0.30043).withKV(0.36869).withKA(0.0050201)
                    .withKP(0.44095).withKI(0).withKD(0)
            );
        
        PhoenixUtil.assertOk(upShooter, () -> upShooter.getConfigurator().apply(upShooterConfig));

        TalonFXConfiguration downShooterConfig = new TalonFXConfiguration()
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(kSensorToDownShooterRatio)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKS(0.22682).withKV(0.36803).withKA(0.011237)
                    .withKP(0.16811).withKI(0).withKD(0)
            )
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(50)
                    .withMotionMagicJerk(500)
            );
        
        PhoenixUtil.assertOk(downShooter, () -> downShooter.getConfigurator().apply(downShooterConfig));

        angleMotor.setPosition(0.0);

        var upShooterSim = upShooter.getSimState();
        upShooterSim.Orientation = ChassisReference.CounterClockwise_Positive;
        upShooterSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);

        var angleSim = angleMotor.getSimState();
        angleSim.Orientation = ChassisReference.CounterClockwise_Positive;
        angleSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    public void set(double upShooterRPS, double downShooterRPS) {
        upShooter.setControl(upShooterRequeset.withVelocity(upShooterRPS));
        downShooter.setControl(downShooterRequest.withVelocity(downShooterRPS));
    }

    public void setAngle(double angleRotations) {
        angleMotor.setControl(angleRequest.withPosition(angleRotations));
    }

    public boolean atTarget() {
        return atTargetSpeed() && atTargetAngle();
    }

    public boolean atTargetSpeed() {
        return upShooter.getMotionMagicAtTarget().getValue() && downShooter.getMotionMagicAtTarget().getValue();
    }

    public boolean atTargetAngle() {
        return Math.abs(angleMotor.getClosedLoopError().getValueAsDouble()) < 0.002;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutineToApply.dynamic(direction);
    }

    public void registerTelemetry(DoubleConsumer telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        if (telemetry != null) {
            telemetry.accept(Units.rotationsToRadians(angleMotor.getPosition().getValueAsDouble()));
        }
        SmartDashboard.putNumber("UpShooterSpeed", upShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("DownShooterSpeed", downShooter.getVelocity().getValueAsDouble());
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