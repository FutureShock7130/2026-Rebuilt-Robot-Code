package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants.angleMap;
import static frc.robot.Constants.ShooterConstants.downSpeedMap;
import static frc.robot.Constants.ShooterConstants.timeOfFlightMap;
import static frc.robot.Constants.ShooterConstants.upSpeedMap;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShotCompensator {
    private static final double PHASE_DELAY_SEC = 0.04;
    private static final double MIN_DIST = 0.2;
    private static final double MAX_DIST = 8.0;
    private static final double HEADING_COMPENSATION_FACTOR = 0.5;

    private final LinearFilter vxFilter = LinearFilter.singlePoleIIR(0.2, 0.02);
    private final LinearFilter vyFilter = LinearFilter.singlePoleIIR(0.2, 0.02);

    public record SolvingParameters(
        Pose2d robotPose,
        ChassisSpeeds robotSpeeds,
        Translation2d targetLocation
    ) {}

    public record FiringSolution(
        double shooterAngle,
        double shooterUpSpeed,
        double shooterDownSpeed,
        double effectiveDistance,
        boolean isReachable
    ) {}

    public FiringSolution solve(SolvingParameters params) {
        // Phase Delay Compensation
        Pose2d predictedPose = predictPose(params.robotPose(), params.robotSpeeds(), PHASE_DELAY_SEC);

        // Dynamic Distance Calculation
        double effectiveDistance = calculateEffectiveDistance(predictedPose, params.robotSpeeds(), params.targetLocation());

        boolean isReachable = effectiveDistance >= MIN_DIST && effectiveDistance <= MAX_DIST;

        double targetAngle = angleMap.get(effectiveDistance);
        double targetUpSpeed = upSpeedMap.get(effectiveDistance);
        double targetDownSpeed = downSpeedMap.get(effectiveDistance);

        return new FiringSolution(targetAngle, targetUpSpeed, targetDownSpeed, effectiveDistance, isReachable);
    }

    private Pose2d predictPose(Pose2d currentPose, ChassisSpeeds speeds, double phaseDelay) {
        return currentPose.exp(
            new Twist2d(
                speeds.vxMetersPerSecond * phaseDelay,
                speeds.vyMetersPerSecond * phaseDelay,
                speeds.omegaRadiansPerSecond * phaseDelay
            )
        );
    }

    // use iterative lookahead to find the effective distance considering robot
    // movement and fuel flight time
    // Tune this factor. Start at 0.3. 
    // If it STILL shoots too far when driving towards the target, increase to 0.5 or 0.6.
    private static final double DISTANCE_COMP_FACTOR = 0.5;

    private double calculateEffectiveDistance(Pose2d robotPose, ChassisSpeeds speeds, Translation2d target) {
        Translation2d robotTrans = robotPose.getTranslation();
        double currentDistance = target.getDistance(robotTrans);
        
        double smoothVx = vxFilter.calculate(speeds.vxMetersPerSecond);
        double smoothVy = vyFilter.calculate(speeds.vyMetersPerSecond);

        Translation2d toTarget = target.minus(robotTrans);
        Rotation2d angleToTarget = toTarget.getAngle();

        double radialVelocity = (smoothVx * angleToTarget.getCos()) + (smoothVy * angleToTarget.getSin());

        double baseToF = timeOfFlightMap.get(currentDistance);
        
        // If moving TOWARDS the target (radialVelocity > 0), this offset will be POSITIVE.
        double distanceOffset = radialVelocity * baseToF * DISTANCE_COMP_FACTOR;

        double effectiveDistance = currentDistance - distanceOffset;

        return Math.max(MIN_DIST, Math.min(effectiveDistance, MAX_DIST));
    }

    public Rotation2d calculateCompensatedHeading(Pose2d robotPose, ChassisSpeeds fieldSpeeds, Translation2d target) {
        double smoothVx = vxFilter.calculate(fieldSpeeds.vxMetersPerSecond);
        double smoothVy = vyFilter.calculate(fieldSpeeds.vyMetersPerSecond);

        double effectiveDist = calculateEffectiveDistance(robotPose, fieldSpeeds, target);
        double tof = timeOfFlightMap.get(effectiveDist);

        double vX = target.getX() - (smoothVx * tof * HEADING_COMPENSATION_FACTOR);
        double vY = target.getY() - (smoothVy * tof * HEADING_COMPENSATION_FACTOR);
        
        return new Translation2d(vX, vY).minus(robotPose.getTranslation()).getAngle();
    }
}