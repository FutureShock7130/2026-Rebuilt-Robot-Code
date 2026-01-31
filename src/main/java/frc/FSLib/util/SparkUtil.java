package frc.FSLib.util;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;

public class SparkUtil {
  /**
   * Apply the config to the SparkMax. Show error on DS and riolog if failed.
   * @param sparkMax the SparkMax to apply the config
   * @param config the config to apply to the SparkMax
   */
  public static void applyConfig(SparkMax sparkMax, SparkMaxConfig config) {
    assertOk(sparkMax, () -> sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  /**
   * Apply the command to the SparkMax. Show error on DS and riolog if failed.
   * @param sparkMax the SparkMax to apply the command
   * @param command the command to apply to the SparkMax
   */
  public static void assertOk(SparkMax sparkMax, Supplier<REVLibError> command) {
    REVLibError error = command.get();
    if (error != REVLibError.kOk) {
      DriverStation.reportError("can't apply configuration to SparkMax with ID " + sparkMax.getDeviceId(), true);
    }
  }

  public static enum StatusFrames {
    kAbslouteEncoder,
    kAnalogSensor,
    kAlternateEncoder;
  }

  private static int kOptimizedFrameRateMs = 500;

  /**
   * Generate a SingalsConfig with optimized AbslouteEncoder, AltEncoder, and AnalogSensor StatusFrames
   * @return the optimized SignalsConfig
   */
  public static SignalsConfig optimizedSignalConfig() {
    return optimizedSignalConfig(StatusFrames.kAbslouteEncoder, StatusFrames.kAlternateEncoder, StatusFrames.kAnalogSensor);
  }

  /**
   * Generate a SingalsConfig with optimized StatusFrame rate(s)
   * @param frames which StatusFrame to optimized
   * @return the optmized SignalsConfig
   */
  public static SignalsConfig optimizedSignalConfig(StatusFrames ...frames) {
    SignalsConfig config = new SignalsConfig();
    for (StatusFrames frame : frames) {
      switch (frame) {
        case kAbslouteEncoder:
          config
            .absoluteEncoderPositionPeriodMs(kOptimizedFrameRateMs)
            .absoluteEncoderVelocityPeriodMs(kOptimizedFrameRateMs);
          break;
        case kAlternateEncoder:
          config
            .externalOrAltEncoderPosition(kOptimizedFrameRateMs)
            .externalOrAltEncoderVelocity(kOptimizedFrameRateMs);
          break;
        case kAnalogSensor:
          config
            .analogPositionPeriodMs(kOptimizedFrameRateMs)
            .analogVelocityPeriodMs(kOptimizedFrameRateMs)
            .analogVoltagePeriodMs(kOptimizedFrameRateMs);
          break;
      }
    }
    return config;
  }
}
