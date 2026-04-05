package frc.robot.commands;

import static frc.robot.Constants.kMaxAngularRate;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.util.AllianceFlipUtil;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignTower extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier xSpeedSupplier, ySpeedSupplier, rotSpeedSupplier;

    private final SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle()
        .withMaxAbsRotationalRate(kMaxAngularRate)
        .withHeadingPID(12, 0, 0.3)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private Translation2d rightClimbPos = new Translation2d(1.15, 3.0);
    private Translation2d leftClimbPos = new Translation2d(0.95, 4.5);
    private double targetAngleDeg = 0.0;
    private double xSpeed, ySpeed;
    private final Timer missingTargetTimer = new Timer();

    public AlignTower(
        CommandSwerveDrivetrain drivetrain,
        DoubleSupplier xSpeedSupplier,
        DoubleSupplier ySpeedSupplier,
        DoubleSupplier rotSpeedSupplier
    ) {
        this.drivetrain = drivetrain;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.rotSpeedSupplier = rotSpeedSupplier;
        addRequirements(drivetrain);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // LimelightHelpers.setPriorityTagID("limelight-left", AllianceFlipUtil.isRedAlliance() ? 16 : 32);
        missingTargetTimer.restart();
    }

    @Override
    public void execute() {
        // targetAngleDeg += rotSpeedSupplier.getAsDouble();
        // xSpeed = -xSpeedSupplier.getAsDouble();
        // ySpeed = -ySpeedSupplier.getAsDouble();
        // if (LimelightHelpers.getTV("limelight-left")) {
        //     double[] positions = LimelightHelpers.getBotPose_TargetSpace("limelight-left");
        //     xSpeed += (positions[2] - 0.7) * 0.2;
        //     ySpeed += (positions[0] - 0.7) * 0.2;
        // }
        Translation2d curr = drivetrain.getState().Pose.getTranslation();
        Translation2d target = AllianceFlipUtil.flip(AllianceFlipUtil.flipY(drivetrain.getState().Pose.getTranslation().getY()) < 4.035 ? rightClimbPos : leftClimbPos);
        Translation2d error = AllianceFlipUtil.isRedAlliance() ? target.minus(curr).unaryMinus() : target.minus(curr);
        drivetrain.setControl(
            drive.withVelocityX(1.0 * error.getX())
                .withVelocityY(4.5 * error.getY())
                .withTargetDirection(AllianceFlipUtil.isRedAlliance() ? Rotation2d.fromDegrees(target == AllianceFlipUtil.flip(rightClimbPos) ? targetAngleDeg + 180 : targetAngleDeg) : Rotation2d.fromDegrees(target == AllianceFlipUtil.flip(rightClimbPos) ? targetAngleDeg : targetAngleDeg + 180))
        );
    }

    @Override
    public boolean isFinished() {
        return missingTargetTimer.hasElapsed(0.2);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(drive);
    }
}
