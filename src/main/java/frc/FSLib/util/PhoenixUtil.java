package frc.FSLib.util;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;

public class PhoenixUtil {
  public static void assertOk(Pigeon2 pigeon, Supplier<StatusCode> command) {
    StatusCode status = command.get();
    if (status != StatusCode.OK) {
      DriverStation.reportError("can't apply configuration to pigeon with ID " + pigeon.getDeviceID(), true);
    }
  }

  public static void assertOk(CANcoder cancoder, Supplier<StatusCode> command) {
    StatusCode status = command.get();
    if (status != StatusCode.OK) {
      DriverStation.reportError("can't apply configuration to CANcoder with ID " + cancoder.getDeviceID(), true);
    }
  }

  public static void assertOk(TalonFX talonFX, Supplier<StatusCode> command) {
    StatusCode status = command.get();
    if (status != StatusCode.OK) {
      DriverStation.reportError("can't apply configuration to talonFX with ID " + talonFX.getDeviceID(), true);
    }
  }
}
