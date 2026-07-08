package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera(VisionConstants.kCameraName);
    private final PhotonPoseEstimator photonPoseEstimator;
    private final CommandSwerveDrivetrain drivetrain;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        AprilTagFieldLayout fieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        // 換成你賽季對應的 enum

        photonPoseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.kRobotToCam
        );
    }

    @Override
    public void periodic() {
        Pose2d prevPose = drivetrain.getState().Pose;
        photonPoseEstimator.setReferencePose(prevPose);

        var results = camera.getAllUnreadResults();
        for (var result : results) {
            Optional<EstimatedRobotPose> visionEst = photonPoseEstimator.update(result);
            visionEst.ifPresent(est -> {
                double ambiguity = result.hasTargets()
                    ? result.getBestTarget().getPoseAmbiguity()
                    : 1.0;

                // 過濾掉不可靠的量測
                if (ambiguity > 0.2) return;

                Matrix<N3, N1> stdDevs = getEstimationStdDevs(est);

                drivetrain.addVisionMeasurement(
                    est.estimatedPose.toPose2d(),
                    est.timestampSeconds,
                    stdDevs
                );
            });
        }
    }

    private Matrix<N3, N1> getEstimationStdDevs(EstimatedRobotPose est) {
        int numTags = est.targetsUsed.size();
        double avgDist = est.targetsUsed.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average().orElse(0);

        double xyStdDev = 0.1 * Math.pow(avgDist, 2) / numTags;
        double thetaStdDev = 0.3 * Math.pow(avgDist, 2) / numTags;

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }
}
