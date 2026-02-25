package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.kHubLocation;
import static frc.robot.Constants.FieldConstants.kTransportTarget_Left;
import static frc.robot.Constants.FieldConstants.kTransportTarget_Right;
import static frc.robot.Constants.FieldConstants.kLeftTrenchStartPoint_Blue;
import static frc.robot.Constants.FieldConstants.kRightTrenchStartPoint_Blue;


import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    private final BooleanSupplier doAimSupplier, doShootSupplier, doTransportSupplier;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeeds;

    /** Command for shooting during auto */
    public ShooterIndexer(
            Shooter shooter,
            Indexer indexer,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> chassisSpeeds) {
        this(shooter, indexer, () -> true, () -> true, () -> false, robotPoseSupplier, chassisSpeeds);
    }

    public ShooterIndexer(
            Shooter shooter,
            Indexer indexer,
            BooleanSupplier doAimSupplier,
            BooleanSupplier doShootSupplier,
            BooleanSupplier doTransportSupplier,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> chassisSpeeds) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.doAimSupplier = doAimSupplier;
        this.doShootSupplier = doShootSupplier;
        this.doTransportSupplier = doTransportSupplier;
        this.robotPoseSupplier = robotPoseSupplier;
        this.chassisSpeeds = chassisSpeeds;

        addRequirements(shooter, indexer);
    }

    @Override
    public void execute() {
        if (doAimSupplier.getAsBoolean()) {
            SolvingParameters solvingParameters = new SolvingParameters(robotPoseSupplier.get(), chassisSpeeds.get(),
                    AllianceFlipUtil.flip(kHubLocation));
            FiringSolution solution = compensator.solve(solvingParameters);
            
            shooter.setAngle(solution.shooterAngle());
            shooter.setSpeed(solution.shooterUpSpeed(), solution.shooterDownSpeed());

            if (doShootSupplier.getAsBoolean() && shooter.atTargetAngle() && shooter.atTargetSpeed()
                    && solution.isReachable()) {
                indexer.set(1.0, 1.0);
            } else {
                indexer.set(0.0, 0.0);
            }
        } else if (doTransportSupplier.getAsBoolean()) {
            SolvingParameters solvingParameters_Transport= new SolvingParameters(robotPoseSupplier.get(), chassisSpeeds.get(),
                    AllianceFlipUtil.flip(getCloserTrenchStartPoint() == AllianceFlipUtil.flip(kLeftTrenchStartPoint_Blue)
                            ? kTransportTarget_Left
                            : kTransportTarget_Right));
            FiringSolution solution_Transport = compensator.solve(solvingParameters_Transport);
            shooter.setAngle(solution_Transport.shooterAngle());
            shooter.setSpeed(solution_Transport.shooterUpSpeed() - 8, solution_Transport.shooterDownSpeed() - 8);
            if (doShootSupplier.getAsBoolean() && shooter.atTargetAngle() && shooter.atTargetSpeed()
                    && solution_Transport.isReachable()) {
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

    private Translation2d getCloserTrenchStartPoint() {
        Translation2d robotTranslation =robotPoseSupplier.get().getTranslation();

        Translation2d actualLeftStart = AllianceFlipUtil.flip(kLeftTrenchStartPoint_Blue);
        Translation2d actualRightStart = AllianceFlipUtil.flip(kRightTrenchStartPoint_Blue);

        double robotToLeftTrench = robotTranslation.getDistance(actualLeftStart);
        double robotToRightTrench = robotTranslation.getDistance(actualRightStart);

        return robotToLeftTrench < robotToRightTrench ? actualLeftStart : actualRightStart;
    }
}
