package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.kHubLocation;
import static frc.robot.Constants.FieldConstants.kLeftTransportTarget;
import static frc.robot.Constants.FieldConstants.kRightTransportTarget;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.util.AllianceFlipUtil;
import frc.robot.LEDCenter;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.shooter.ShotCompensator;
import frc.robot.subsystems.shooter.ShotCompensator.FiringSolution;
import frc.robot.subsystems.shooter.ShotCompensator.SolvingParameters;

public class ShooterIndexer extends Command {
    private final Shooter shooter;
    private final Indexer indexer;

    private final BooleanSupplier doAimSupplier, doShootSupplier;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    private final ShotCompensator compensator = new ShotCompensator();

    /** Command for shooting during auto */
    public ShooterIndexer(
        Shooter shooter,
        Indexer indexer,
        Supplier<Pose2d> robotPoseSupplier,
        Supplier<ChassisSpeeds> chassisSpeeds
    ) {
        this(shooter, indexer, () -> true, () -> true, robotPoseSupplier, chassisSpeeds);
    }

    public ShooterIndexer(
        Shooter shooter,
        Indexer indexer,
        BooleanSupplier doAimSupplier,
        BooleanSupplier doShootSupplier,
        Supplier<Pose2d> robotPoseSupplier,
        Supplier<ChassisSpeeds> chassisSpeedsSupplier
    ) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.doAimSupplier = doAimSupplier;
        this.doShootSupplier = doShootSupplier;
        this.robotPoseSupplier = robotPoseSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;

        addRequirements(shooter, indexer);
    }

    @Override
    public void execute() {
        if (doAimSupplier.getAsBoolean()) {
            LEDCenter.getInstance().setAiming();
            // Switch aiming target between HUB and transport target based on robot's position
            Translation2d robotTranslation = robotPoseSupplier.get().getTranslation();
            Translation2d aimingTarget;
            if ((robotTranslation.getX() - AllianceFlipUtil.flipX(0)) * (robotTranslation.getX() - AllianceFlipUtil.flipX(4.028)) < 0) { // in alliance zones
                aimingTarget = kHubLocation;
            } else {
                aimingTarget = AllianceFlipUtil.flipY(robotTranslation.getY()) < 4.035 ? kRightTransportTarget : kLeftTransportTarget;
            }
            SolvingParameters solvingParameters = new SolvingParameters(robotPoseSupplier.get(), chassisSpeedsSupplier.get(), AllianceFlipUtil.flip(aimingTarget));
            FiringSolution solution = compensator.solve(solvingParameters);
            
            if ((robotTranslation.getX() - AllianceFlipUtil.flipX(0)) * (robotTranslation.getX() - AllianceFlipUtil.flipX(4.028)) < 0) { // in alliance zones
                shooter.setAngle(solution.shooterAngle());
                shooter.setSpeed(solution.shooterUpSpeed(), solution.shooterDownSpeed());
            } else {
                shooter.setAngle(solution.shooterAngle());
                shooter.setSpeed(solution.shooterUpSpeed() - 8, solution.shooterDownSpeed() - 16);
            }
            
            if (doShootSupplier.getAsBoolean() && solution.isReachable()) {
                indexer.set(IndexerConstants.kIndexingSpeed, IndexerConstants.kIndexingSpeed);
                LEDCenter.getInstance().setShooting();
            } else {
                indexer.set(0.0, 0.0);
            }
        } else {
            shooter.setAngle(0.0);
            shooter.setSpeed(0.0, 0.0);
            indexer.set(0.0, 0.0);
            LEDCenter.getInstance().setDefault();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setAngle(0.0);
        shooter.setSpeed(0.0, 0.0);
        indexer.set(0.0, 0.0);
    }
}
