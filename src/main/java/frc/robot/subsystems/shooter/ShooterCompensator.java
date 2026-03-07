package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.FSLib.util.AllianceFlipUtil;
import static frc.robot.Constants.ShooterConstants.angleMap;
import static frc.robot.Constants.ShooterConstants.upSpeedMap;
import static frc.robot.Constants.ShooterConstants.downSpeedMap;
import static frc.robot.Constants.ShooterConstants.timeOfFlightMap;

public class ShooterCompensator {

    private static final double PHASE_DELAY_SEC = 0.04;
    private static final double MIN_DIST = 0.0;
    private static final double MAX_DIST = 8.0;
    private static final int LOOKAHEAD_ITERATIONS = 10;
    private static final double VELOCITY_MULTIPLIER = 0.6;

    private final LinearFilter vxFilter = LinearFilter.singlePoleIIR(0.2, 0.02);
    private final LinearFilter vyFilter = LinearFilter.singlePoleIIR(0.2, 0.02);

    public ShooterCompensator() {
    }

    public record SolvingParameters(
            Pose2d robotPose,
            ChassisSpeeds robotSpeeds, // MUST be Robot-Relative
            Translation2d targetLocation) {
    }

    public record FiringSolution(
            double shooterAngle,
            double shooterUpSpeed,
            double shooterDownSpeed,
            double effectiveDistance,
            Rotation2d targetHeading,
            boolean isReachable) {
    }

    public FiringSolution solve(SolvingParameters params) {
        Pose2d predictedPose = predictPose(params.robotPose(), params.robotSpeeds(), PHASE_DELAY_SEC);

        Rotation2d currentHeading = predictedPose.getRotation();
        double fieldVx = (params.robotSpeeds().vxMetersPerSecond * currentHeading.getCos())
                - (params.robotSpeeds().vyMetersPerSecond * currentHeading.getSin());
        double fieldVy = (params.robotSpeeds().vxMetersPerSecond * currentHeading.getSin())
                + (params.robotSpeeds().vyMetersPerSecond * currentHeading.getCos());

        double smoothFieldVx = vxFilter.calculate(fieldVx);
        double smoothFieldVy = vyFilter.calculate(fieldVy);

        Translation2d robotTranslation = predictedPose.getTranslation();
        double lookaheadDistance = params.targetLocation().getDistance(robotTranslation);
        Translation2d lookaheadRobotTranslation = robotTranslation;

        for (int i = 0; i < LOOKAHEAD_ITERATIONS; i++) {
            double timeOfFlight = timeOfFlightMap.get(lookaheadDistance);
            
            double offsetX = smoothFieldVx * timeOfFlight * VELOCITY_MULTIPLIER;
            double offsetY = smoothFieldVy * timeOfFlight * VELOCITY_MULTIPLIER;
            
            lookaheadRobotTranslation = robotTranslation.plus(new Translation2d(offsetX, offsetY));
            lookaheadDistance = params.targetLocation().getDistance(lookaheadRobotTranslation);
        }

        boolean isReachable = lookaheadDistance >= MIN_DIST && lookaheadDistance <= MAX_DIST;

        double targetAngle = angleMap.get(lookaheadDistance);
        double targetUpSpeed = upSpeedMap.get(lookaheadDistance);
        double targetDownSpeed = downSpeedMap.get(lookaheadDistance);

        Rotation2d targetHeading = params.targetLocation().minus(lookaheadRobotTranslation).getAngle();
        targetHeading = AllianceFlipUtil.flip(targetHeading);

        return new FiringSolution(
                targetAngle,
                targetUpSpeed,
                targetDownSpeed,
                lookaheadDistance,
                targetHeading,
                isReachable);
    }

    private Pose2d predictPose(Pose2d currentPose, ChassisSpeeds robotSpeeds, double phaseDelay) {
        return currentPose.exp(new Twist2d(
                robotSpeeds.vxMetersPerSecond * phaseDelay,
                robotSpeeds.vyMetersPerSecond * phaseDelay,
                robotSpeeds.omegaRadiansPerSecond * phaseDelay));
    }
}