package frc.FSLib.util;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceFlipUtil {
  public static double flipX(double x) {
    return shouldFlip() ? FlippingUtil.fieldSizeX - x : x;
  }

  public static double flipY(double y) {
    return shouldFlip() ? FlippingUtil.fieldSizeY - y : y;
  }

  public static Pose2d flip(Pose2d pose) {
    return shouldFlip() ? FlippingUtil.flipFieldPose(pose) : pose;
  }

  public static Pose3d flip(Pose3d pose) {
    return new Pose3d(flip(pose.getTranslation()), flip(pose.getRotation()));
  }

  public static Translation2d flip(Translation2d translation) {
    return shouldFlip() ? FlippingUtil.flipFieldPosition(translation) : translation;
  }

  public static Translation3d flip(Translation3d translation) {
    return new Translation3d(flipX(translation.getX()), flipY(translation.getY()), translation.getZ());
  }

  public static Rotation2d flip(Rotation2d rotation) {
    return shouldFlip() ? FlippingUtil.flipFieldRotation(rotation) : rotation;
  }

  public static Rotation3d flip(Rotation3d rotation) {
    return shouldFlip() ? rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI)) : rotation;
  }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }

}