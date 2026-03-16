package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeConstants.*;

import java.util.function.DoubleConsumer;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FSLib.util.PhoenixUtil;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final CANcoder angleEncoder = new CANcoder(kIntakeEncoderId, Constants.kCanivoreBus);
    private final TalonFX angleMotor = new TalonFX(kAngleMotorId, Constants.kCanivoreBus);
    private final TalonFX intakeMotor = new TalonFX(kIntakeMotorId, Constants.kCanivoreBus);

    private final DynamicMotionMagicVoltage angleRequest = new DynamicMotionMagicVoltage(0.0, 0.5, 2.5).withEnableFOC(true);

    private DoubleConsumer telemetry;

    private static final double kGearRatio = kSensorToAngleRatio;
    private final DCMotorSim m_motorSimModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 0.02, kGearRatio
        ),
        DCMotor.getKrakenX60Foc(1)
    );


    public Intake() {
        CANcoderConfiguration angleEncoderConfig = new CANcoderConfiguration();
        angleEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        angleEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        angleEncoderConfig.MagnetSensor.withMagnetOffset(Rotations.of(0.0));
        angleEncoder.getConfigurator().apply(angleEncoderConfig);
        PhoenixUtil.assertOk(angleEncoder, () -> angleEncoder.getConfigurator().apply(angleEncoderConfig));

        TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
        intakeMotorConfig.MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        intakeMotorConfig.CurrentLimits
            .withStatorCurrentLimit(40)
            .withSupplyCurrentLimit(20)
            .withSupplyCurrentLowerLimit(20)
            .withSupplyCurrentLowerTime(0.0);
        intakeMotorConfig.withSlot0(
                new Slot0Configs()
                    .withKG(0.0).withKS(0.39397).withKV(2.6341).withKA(0.1969)
                    .withKP(3.4432).withKI(0.0).withKD(0)
            );

        PhoenixUtil.assertOk(intakeMotor, () -> intakeMotor.getConfigurator().apply(intakeMotorConfig));

        TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
        angleMotorConfig
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(80)
                    .withSupplyCurrentLimit(30)
                    .withSupplyCurrentLowerLimit(30)
                    .withSupplyCurrentLowerTime(0.0)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(1.0)
                    .withFeedbackRemoteSensorID(angleEncoder.getDeviceID())
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    .withRotorToSensorRatio(kSensorToAngleRatio)
            )
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(2.5)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKG(0.02).withKS(0.02).withKV(2.5).withKA(0.02)
                    .withKP(16.0).withKI(0.15).withKD(0.57)
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withGravityArmPositionOffset(kAngleMax)
            )
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(kAngleMin)
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(kAngleMax)
            );

        PhoenixUtil.assertOk(angleMotor, () -> angleMotor.getConfigurator().apply(angleMotorConfig));

        var motorSimState = angleMotor.getSimState();
        motorSimState.Orientation = ChassisReference.Clockwise_Positive;
    }

    public Command intake() {
        return this.startEnd(
            () -> {
                angleMotor.setControl(angleRequest.withPosition(kIntakeAngle));
                intakeMotor.set(kIntakeSpeed);
            },
            () -> {
                angleMotor.setControl(angleRequest.withPosition(kIntakeAngle));
                intakeMotor.set(0);
            }
        ).withName("IntakeIntake");
    }

    public Command outTake() {
        return this.startEnd(
            () -> {
                angleMotor.setControl(angleRequest.withPosition(kIntakeAngle));
                intakeMotor.set(-kIntakeSpeed);
            },
            () -> {
                angleMotor.setControl(angleRequest.withPosition(kIntakeAngle));
                intakeMotor.set(0);
            }
        ).withName("IntakeOutTake");
    }

    public Command toDefaultState() {
        return this.run(
            () -> {
                angleMotor.setControl(angleRequest.withPosition(kDefaultAngle));
                intakeMotor.set(0);
            }
        ).withName("IntakeToDefaultState");
    }

    public Command toFeedingState() {
        return this.run(
            () -> {
                angleMotor.setControl(angleRequest.withPosition(kDefaultAngle));
                intakeMotor.set(0.5);
            }
        ).withName("IntakeToDefaultState");
    }

    public void registerTelemetry(DoubleConsumer telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        if (telemetry != null) {
            telemetry.accept(angleMotor.getPosition().getValueAsDouble());
        }
    }

    @Override
    public void simulationPeriodic() {
        var talonFXSim = angleMotor.getSimState();

        // set the supply voltage of the TalonFX
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
        m_motorSimModel.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(kGearRatio));
        talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(kGearRatio));
    }
}
