package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.kHubLocation;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.util.AllianceFlipUtil;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterCompensator;
import frc.robot.subsystems.shooter.ShooterCompensator.FiringSolution;
import frc.robot.subsystems.shooter.ShooterCompensator.SolvingParameters;

public class ShooterIndexer extends Command {
    private final Shooter shooter;
    private final Indexer indexer;
    private final ShooterCompensator compensator = new ShooterCompensator();

    private final BooleanSupplier doAimSupplier, doShootSupplier;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeeds;

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
        Supplier<ChassisSpeeds> chassisSpeeds
    ) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.doAimSupplier = doAimSupplier;
        this.doShootSupplier = doShootSupplier;
        this.robotPoseSupplier = robotPoseSupplier;
        this.chassisSpeeds = chassisSpeeds;

        addRequirements(shooter, indexer);
    }

    @Override
    public void execute() {
        if (doAimSupplier.getAsBoolean()) {
            SolvingParameters solvingParameters = new SolvingParameters(robotPoseSupplier.get(), chassisSpeeds.get(), AllianceFlipUtil.flip(kHubLocation));
            FiringSolution solution = compensator.solve(solvingParameters);

            shooter.setAngle(solution.shooterAngle());
            shooter.setSpeed(solution.shooterUpSpeed(), solution.shooterDownSpeed());

            SmartDashboard.putBoolean("atTargetSpeed", shooter.atTargetSpeed());
            SmartDashboard.putBoolean("atTargetAngle", shooter.atTargetAngle());
            if (doShootSupplier.getAsBoolean() && shooter.atTargetSpeed() && solution.isReachable()) {
                indexer.set(1.0, 1.0);
            } else {
                indexer.set(0.0, 0.0);
            }
        } else {
            shooter.setAngle(0.0);
            shooter.setSpeed(0.0, 0.0);
            indexer.set(0.0, 0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setAngle(0.0);
        shooter.setSpeed(0.0, 0.0);
        indexer.set(0.0, 0.0);
    }
}
