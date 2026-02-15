package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

public class ShooterCompensator {

    private final InterpolatingDoubleTreeMap angleMap = Constants.ShooterConstants.angleMap;
    private final InterpolatingDoubleTreeMap upSpeedMap = Constants.ShooterConstants.upSpeedMap;
    private final InterpolatingDoubleTreeMap downSpeedMap = Constants.ShooterConstants.downSpeedMap;
    private final InterpolatingDoubleTreeMap timeOfFlightMap = Constants.ShooterConstants.timeOfFlightMap;

    private static final double PHASE_DELAY_SEC = 0.04;
    private static final double MIN_DIST = 0.2;
    private static final double MAX_DIST = 6.0;

    public ShooterCompensator() {}

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
        double effectiveDistance = calculateEffectiveDistance(
                predictedPose,
                params.robotSpeeds(),
                params.targetLocation());

        boolean isReachable = effectiveDistance >= MIN_DIST && effectiveDistance <= MAX_DIST;

        double targetAngle = angleMap.get(effectiveDistance);
        double targetUpSpeed = upSpeedMap.get(effectiveDistance);
        double targetDownSpeed = downSpeedMap.get(effectiveDistance);

        return new FiringSolution(
                targetAngle,
                targetUpSpeed,
                targetDownSpeed,
                effectiveDistance,
                isReachable);
    }

    private Pose2d predictPose(Pose2d currentPose, ChassisSpeeds speeds, double phaseDelay) {
        return currentPose.exp(new Twist2d(
                speeds.vxMetersPerSecond * phaseDelay,
                speeds.vyMetersPerSecond * phaseDelay,
                speeds.omegaRadiansPerSecond * phaseDelay));
    }

    // use iterative lookahead to find the effective distance considering robot movement and fuel flight time
    private double calculateEffectiveDistance(Pose2d robotPose, ChassisSpeeds speeds, Translation2d target) {
        Translation2d robotTrans = robotPose.getTranslation();
        double currentDistance = target.getDistance(robotTrans);

        double lookaheadDistance = currentDistance;

        for (int i = 0; i < 5; i++) {
            double timeOfFlight = timeOfFlightMap.get(lookaheadDistance);

            double virtualDispX = -speeds.vxMetersPerSecond * timeOfFlight;
            double virtualDispY = -speeds.vyMetersPerSecond * timeOfFlight;

            Translation2d virtualRobotPos = robotTrans.plus(new Translation2d(virtualDispX, virtualDispY));

            lookaheadDistance = target.getDistance(virtualRobotPos);
        }

        return lookaheadDistance;
    }
}