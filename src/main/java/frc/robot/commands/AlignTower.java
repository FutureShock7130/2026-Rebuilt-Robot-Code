package frc.robot.commands;

import static frc.robot.Constants.kMaxAngularRate;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.util.AllianceFlipUtil;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignTower extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Climber climber;

    private final SwerveRequest.RobotCentricFacingAngle driveRobot = new SwerveRequest.RobotCentricFacingAngle()
        .withMaxAbsRotationalRate(kMaxAngularRate)
        .withHeadingPID(12, 0, 0.3)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Timer timer = new Timer();

    public AlignTower(CommandSwerveDrivetrain drivetrain, Climber climber) {
        this.drivetrain = drivetrain;
        this.climber = climber;
        addRequirements(drivetrain, climber);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        double targetDeg = drivetrain.getState().Pose.getY() < AllianceFlipUtil.flipY(3.7) ? 0 : 180;
        targetDeg = AllianceFlipUtil.isRedAlliance() ? targetDeg + 180 : targetDeg;
        if (!climber.isAlignedToTower()) {
            drivetrain.setControl(
                driveRobot.withVelocityX(0.7)
                    .withVelocityY(0)
                    .withTargetDirection(Rotation2d.fromDegrees(targetDeg))
            );
            timer.reset();
        } else if (climber.getDistance() > 0.05) {    
            drivetrain.setControl(
                driveRobot.withVelocityX(0)
                    .withVelocityY(0.2)
                    .withTargetDirection(Rotation2d.fromDegrees(targetDeg))
            );
            timer.reset();
        } else {
            drivetrain.setControl(
                driveRobot.withVelocityX(0.4)
                    .withVelocityY(0.4)
                    .withTargetDirection(Rotation2d.fromDegrees(targetDeg))
            );
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }
}
