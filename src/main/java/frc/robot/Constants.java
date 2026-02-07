package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;

import frc.robot.generated.TunerConstants;

public class Constants {

    public static enum RobotState {
        kCycling,
        kClimbing
    }

    public static final CANBus kCanivoreBus = new CANBus("GTX7130");

    public static final double kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double kMaxAngularRate = RotationsPerSecond.of(0.95).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public static final class ClimberConstants {
        public static final int kLeftClimberId = 46;
        public static final int kRightClimberId = 45;

        public static final double kSensorToMechanismRatio = 12;
        public static final double kMechanismToHeighRatio = 0.15708;
        public static final double kPreClimbHeightMeters = 0.1;
        public static final double kClimbHeightMeters = 0.05;

        public static final double kMaxHeightMeters = 0.0;
        public static final double kMinHeightMeters = 0.0;
        // public static final double kMaxPosition = kPreClimbHeightMeters / kMechanismToHeighRatio;
    }

    public static final class IndexerContants {
        public static final int kUpIndexId = 44;
        public static final int kDownIndexId = 43;
    }

    public static final class IntakeConstants {
        public static final int kMotorId = 60;

        public static final double kIntakeSpeed = 1.0;
    }

    public static final class ShooterContants {
        public static final int kAngleId = 56;
        public static final int kUpShooterId = 54;
        public static final int kDownShooterId = 55;

        public static final double kAngleMax = 0.05;
        public static final double kAngleMin = -0.002;

        public static final double kSensorToAngleRatio = 150.0;
        public static final double kSensorToUpShooterRatio = 3.0;
        public static final double kSensorToDownShooterRatio = 3.0;
    }
}
