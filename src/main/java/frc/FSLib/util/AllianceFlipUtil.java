package frc.FSLib.util;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceFlipUtil {
  public static double flipX(double x) {
    return isRedAlliance() ? FlippingUtil.fieldSizeX - x : x;
  }

  public static double flipY(double y) {
    return isRedAlliance() ? FlippingUtil.fieldSizeY - y : y;
  }

  /**
   * Flip a field pose to the other side of the field, maintaining a blue alliance
   * origin. This should be only used for flipping a field-coordinate pose such
   * as the Pose2d of a scoring target.
   * 
   * @param pose The Pose2d object to flip
   * @return The flipped Pose2d object
   */
  public static Pose2d flip(Pose2d pose) {
    return isRedAlliance() ? FlippingUtil.flipFieldPose(pose) : pose;
  }

  /**
   * Flip a field translation to the other side of the field, maintaining a blue alliance
   * origin. This should be only used for flipping a field-coordinate translation, such
   * as the Translation2d of a scoring target.
   * 
   * @param translation The Translation2d object to flip
   * @return The flipped Translation2d object
   */
  public static Translation2d flip(Translation2d translation) {
    return isRedAlliance() ? FlippingUtil.flipFieldPosition(translation) : translation;
  }

  /**
   * Flip a field translation to the other side of the field, maintaining a blue alliance
   * origin. This should be only used for flipping a field-coordinate translation, such
   * as the Translation3d of a scoring target.
   * 
   * @param translation The Translation3d object to flip
   * @return The flipped Translation3d object
   */
  public static Translation3d flip(Translation3d translation) {
    return isRedAlliance() ? switch (FlippingUtil.symmetryType) {
      case kMirrored -> new Translation3d(flipX(translation.getX()), translation.getY(), translation.getZ());
      case kRotational -> new Translation3d(flipX(translation.getX()), flipY(translation.getY()), translation.getZ());
    } : translation;
  }

  /**
   * Flip a field rotation to the other side of the field, maintaining a blue alliance
   * origin. This should be only used for flipping a field-coordinate rotation, such
   * as the Rotation2d of a scoring target.
   * 
   * @param rotation The Rotation2d object to flip
   * @return The flipped Rotation2d object
   */
  public static Rotation2d flip(Rotation2d rotation) {
    return isRedAlliance() ? FlippingUtil.flipFieldRotation(rotation) : rotation;
  }

  /**
   * Convert a field-coordinate Rotation2d to alliance-coordinate Rotation2d
   * <p>
   * An example scenario is that the field-coordinate (blue-alliance-coordinate)
   * Rotation2d is PI/2 (facing left in blue alliance's perspective). In this
   * scenario, the output will be PI/2 if the current alliance is blue, and -PI/2
   * (facing right in red alliance's perspective) if the current alliance is red.
   * <p>
   * This is useful when trying to make the robot facing a field-coordinate
   * target, such as a scoring target.
   * 
   * @param rotation The field-coordinate Rotation2d object
   * @return The alliance-coordinate Rotation2d object
   */
  public static Rotation2d toAllianceCoord(Rotation2d rotation) {
    return isRedAlliance() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  /**
   * Convert a alliance-coordinate Rotation2d to field-coordinate Rotation2d
   * <p>
   * An example scenario is that the red-alliance-coordinate Rotation2d is PI/2
   * (facing left in red alliance's perspective). In this scenario, the output
   * will be -PI/2, which faces the same direction with the original Rotation2d in
   * blue alliance's perspective.
   * <p>
   * This is useful when converting the robot's facing target to the odometry's
   * coordinate system.
   * 
   * @param rotation The alliance-coordinate Rotation2d object
   * @return The field-coordinate Rotation2d object
   */
  public static Rotation2d toFieldCoord(Rotation2d rotation) {
    return isRedAlliance() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }
}