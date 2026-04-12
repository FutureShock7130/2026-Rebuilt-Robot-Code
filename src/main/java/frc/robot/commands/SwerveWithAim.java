package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.util.AllianceFlipUtil;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SwerveWithAim extends Command {
    private final CommandSwerveDrivetrain drivetrain;

    private final DoubleSupplier xSpeedSupplier, ySpeedSupplier, rotSpeedSupplier;
    private final BooleanSupplier doAimSupplier;

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 1 of a rotation per second max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentricFacingAngle driveAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1) // Add a 10% deadband
            .withMaxAbsRotationalRate(MaxAngularRate)
            .withHeadingPID(10, 0, 0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Translation2d target = new Translation2d(4.625, 4.034);

    public SwerveWithAim(
        CommandSwerveDrivetrain drivetrain,
        DoubleSupplier xSpeedSupplier,
        DoubleSupplier ySpeedSupplier,
        DoubleSupplier rotSpeedSupplier,
        BooleanSupplier doAimSupplier
    ) {
        this.drivetrain = drivetrain;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.rotSpeedSupplier = rotSpeedSupplier;
        this.doAimSupplier = doAimSupplier;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {

        Pose2d curr = drivetrain.getState().Pose;

        if (
            (curr.getX() - AllianceFlipUtil.flipX(0)) * (curr.getX() - AllianceFlipUtil.flipX(4.028)) < 0 // in alliance zon
            && doAimSupplier.getAsBoolean()
        ) {
            Rotation2d val = AllianceFlipUtil.flip(target).minus(curr.getTranslation()).getAngle();

            drivetrain.setControl(
                driveAngle.withVelocityX(xSpeedSupplier.getAsDouble())
                    .withVelocityY(ySpeedSupplier.getAsDouble())
                    .withTargetDirection(val)
            );
        } else {
            drivetrain.setControl(
                drive.withVelocityX(xSpeedSupplier.getAsDouble())
                    .withVelocityY(ySpeedSupplier.getAsDouble())
                    .withRotationalRate(rotSpeedSupplier.getAsDouble())
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(null);
    }
}
