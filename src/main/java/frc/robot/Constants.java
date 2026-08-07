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
    public static final double kMaxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond);

    public static final class FieldConstants {
        public static final Translation2d kHubLocation = new Translation2d(4.625, 4.034);
        public static final Translation2d kLeftTransportTarget = new Translation2d(1.387, 6.487);
        public static final Translation2d kRightTransportTarget = new Translation2d(1.387, 1.367);
        public static final Translation2d kLeftTrenchStartPoint = new Translation2d(4.6, 7.47);
        public static final Translation2d kRightTrenchStartPoint = new Translation2d(4.6, 0.6);
        public static final Translation2d[] kTrenchPoses = new Translation2d[]{kRightTrenchStartPoint, kLeftTrenchStartPoint};
    }

    public static final class ClimberConstants {
        public static final int kMotorId = 46;
        public static final int kCANRangeId = 47;

        public static final double kSensorToMechanismRatio = 60;
        public static final double kMechanismToHeighRatio = 0.11401528;
        public static final double kPreClimbHeightMeters = 0.242;
        public static final double kClimbHeightMeters = 0.0;

        public static final double kMaxHeightMeters = 0.243;
        public static final double kMinHeightMeters = 0.0;
    }

    public static final class IndexerConstants {
        public static final int kUpIndexId = 44;
        public static final int kDownIndexId = 43;

        public static final double kIndexingSpeed = 0.63;
    }

    public static final class IntakeConstants {
        public static final int kAngleMotorId = 61;
        public static final int kIntakeMotorId = 60;
        public static final int kAngleCancoderId = 62;
        
        public static final double kAngleMax = -0.1;
        public static final double kAngleMin =  -0.347412;  

        public static final double kRotorToSensorRatio = 60.0;

        public static final double kIntakeAngle =  -0.347412;
        public static final double kIntakeSpeed = 0.48;
        public static final double kDefaultAngle = -0.08;

        //public static final double kAngleCancoderOffset = -0.317626953125;
        public static final double kAngleCancoderOffset = 0;
    }

    public static final class ShooterConstants {
        public static final int kAngleId = 56;
        public static final int kUpShooterId = 54;
        public static final int kDownShooterId = 55;
        public static final int kDownShooter2Id = 62;

        public static final double kAngleMax = 0.055;
        public static final double kAngleMin = 0;

        public static final double kSensorToAngleRatio = 162.0;
        public static final double kSensorToUpShooterRatio = 3.0;
        public static final double kSensorToDownShooterRatio = 0.5;

        public static final InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap upSpeedMap = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap downSpeedMap = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

        static {
            //green
            // angleMap.put(0.0, 0.02);
            // angleMap.put(1.5, 0.02);
            // angleMap.put(2.0, 0.035);
            // angleMap.put(2.5, 0.0375);
            // angleMap.put(3.0, 0.045);
            // angleMap.put(4.0, 0.052);
            //angleMap.put(4.5, 0.052);
            //angleMap.put(5.0, 0.052);
            //angleMap.put(5.5, 0.052);
            //angleMap.put(6.0, 0.052);
            //angleMap.put(999.0, 0.052);

            //blue
            angleMap.put(0.0, 0.023);
            angleMap.put(1.5, 0.023);
            angleMap.put(2.75, 0.04);
            angleMap.put(3.5, 0.047);
            angleMap.put(4.5, 0.047);
            angleMap.put(5.5, 0.055);
            angleMap.put(999.0, 0.055);
            //green
            // upSpeedMap.put(0.0, 22.0);
            // upSpeedMap.put(1.5, 23.0);
            // upSpeedMap.put(2.0, 24.0);
            // upSpeedMap.put(3.0, 24.0);
            // upSpeedMap.put(4.0, 24.0);
            // upSpeedMap.put(5.0, 25.0);

            //upSpeedMap.put(6.0, 28.0);
            //upSpeedMap.put(999.0, 28.0);

            //blue
            upSpeedMap.put(0.0, 24.0);
            upSpeedMap.put(1.5, 24.0);
            upSpeedMap.put(2.75, 24.0);
            upSpeedMap.put(3.5, 24.0);
            upSpeedMap.put(4.5, 27.0);
            upSpeedMap.put(5.5, 28.0);
            upSpeedMap.put(999.0, 28.0);

            //green
            // downSpeedMap.put(0.0, 33.0);
            // downSpeedMap.put(1.5, 33.0);
            // downSpeedMap.put(2.0, 35.75);
            // downSpeedMap.put(2.5, 36.75);
            // downSpeedMap.put(3.0, 39.45);
            // //downSpeedMap.put(3.5, 42.5);
            // downSpeedMap.put(4.0, 46.25);
            // //downSpeedMap.put(4.5, 50.75);
            // downSpeedMap.put(5.0, 54.0);
            //downSpeedMap.put(6.0, 60.0);
            //downSpeedMap.put(999.0, 60.0);

            //blue
            downSpeedMap.put(0.0, 38.0);
            downSpeedMap.put(1.5, 38.0);
            downSpeedMap.put(2.75, 40.0);
            downSpeedMap.put(3.5, 45.0);
            downSpeedMap.put(4.5, 52.0);
            downSpeedMap.put(5.5, 56.0);
            downSpeedMap.put(999.0, 56.0);

            timeOfFlightMap.put(0.5, 1.2);
            timeOfFlightMap.put(1.0, 1.4);
            timeOfFlightMap.put(2.0, 1.7);
            timeOfFlightMap.put(3.0, 1.9);
            timeOfFlightMap.put(4.0, 2.3);
            timeOfFlightMap.put(6.0, 2.5);
            timeOfFlightMap.put(999.0, 2.5);
        }
    }
}
