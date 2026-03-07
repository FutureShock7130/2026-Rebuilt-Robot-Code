package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.generated.TunerConstants;

public class Constants {
    public static final CANBus kCanivoreBus = new CANBus("GTX7130");

    public static final double kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double kMaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

    public static final class FieldConstants {
        public static final Translation2d kHubLocation = new Translation2d(4.625, 4.034);
        public static final Translation2d kTransportTarget_Left = new Translation2d(1.387, 6.487);
        public static final Translation2d kTransportTarget_Right = new Translation2d(1.387, 1.367);
        public static final Translation2d kLeftTrenchStartPoint_Blue = new Translation2d(3.1, 7.35);
        public static final Translation2d kRightTrenchStartPoint_Blue = new Translation2d(3.1, 0.75);
    }

    public static final class ClimberConstants {
        public static final int kMotorId = 46;

        public static final double kSensorToMechanismRatio = 60;
        public static final double kMechanismToHeighRatio = 0.11401528;
        public static final double kPreClimbHeightMeters = 0.235;
        public static final double kClimbHeightMeters = 0.10;

        public static final double kMaxHeightMeters = 0.24;
        public static final double kMinHeightMeters = 0.0;
    }

    public static final class IndexerConstants {
        public static final int kUpIndexId = 44;
        public static final int kDownIndexId = 43;
    }

    public static final class IntakeConstants {
        public static final int kAngleMotorId = 61;
        public static final int kIntakeMotorId = 60;

        public static final double kAngleMax = 0.0;
        public static final double kAngleMin = -0.28167;

        public static final double kSensorToAngleRatio = 135.0;

        public static final double kIntakeAngle = 0.0;
        public static final double kIntakeSpeed = 1.0;
        public static final double kIntakeVoltage = 12.0;
        public static final double kDefaultAngle = -0.29;
    }

    public static final class ShooterConstants {
        public static final int kAngleId = 56;
        public static final int kUpShooterId = 54;
        public static final int kDownShooterId = 55;
        public static final int kDownShooter2Id = 62;

        public static final double kAngleMax = 0.067;
        public static final double kAngleMin = 0;

        public static final double kSensorToAngleRatio = 150.0;
        public static final double kSensorToUpShooterRatio = 3.0;
        public static final double kSensorToDownShooterRatio = 1.8;

        public static final InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap upSpeedMap = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap downSpeedMap = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

        static {
            angleMap.put(0.0, 0.0);
            angleMap.put(1.4, 0.02);
            angleMap.put(2.3, 0.0325);
            angleMap.put(3.0, 0.0525);
            angleMap.put(4.0, 0.0625);
            angleMap.put(5.0, 0.063);
            angleMap.put(5.5, 0.0635);
            angleMap.put(6.0, 0.065);

            upSpeedMap.put(0.0, 22.0);
            upSpeedMap.put(1.4, 23.0);
            upSpeedMap.put(2.3, 24.0);
            upSpeedMap.put(3.0, 25.0);
            upSpeedMap.put(4.0, 26.0);
            upSpeedMap.put(5.0, 31.0);
            upSpeedMap.put(6.0, 35.0);

            downSpeedMap.put(0.0, 33.5);
            downSpeedMap.put(1.4, 35.0);
            downSpeedMap.put(2.3, 36.0);
            downSpeedMap.put(3.0, 38.5);
            downSpeedMap.put(4.0, 40.5);
            downSpeedMap.put(5.0, 49.0);
            downSpeedMap.put(6.0, 51.0);

            timeOfFlightMap.put(0.5, 1.2);
            timeOfFlightMap.put(1.0, 1.4);
            timeOfFlightMap.put(2.0, 1.7);
            timeOfFlightMap.put(3.0, 1.9);
            timeOfFlightMap.put(4.0, 2.3);
            timeOfFlightMap.put(6.0, 2.5);
        }
    }
}
