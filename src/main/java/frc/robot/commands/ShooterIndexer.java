package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.kHubLocation;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.util.AllianceFlipUtil;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShooterIndexer extends Command {
    private final Shooter shooter;
    private final Indexer indexer;

    private final BooleanSupplier doAimSupplier, doShootSupplier;
    private final Supplier<Pose2d> robotPoseSupplier;

    private final InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap upSpeedMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap downSpeedMap = new InterpolatingDoubleTreeMap();

    /** Command for shooting during auto */
    public ShooterIndexer(
        Shooter shooter,
        Indexer indexer,
        Supplier<Pose2d> robotPoseSupplier
    ) {
        this(shooter, indexer, () -> true, () -> true, robotPoseSupplier);
    }

    public ShooterIndexer(
        Shooter shooter,
        Indexer indexer,
        BooleanSupplier doAimSupplier,
        BooleanSupplier doShootSupplier,
        Supplier<Pose2d> robotPoseSupplier
    ) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.doAimSupplier = doAimSupplier;
        this.doShootSupplier = doShootSupplier;
        this.robotPoseSupplier = robotPoseSupplier;

        addRequirements(shooter, indexer);

        angleMap.put(0.0, 0.0);
        angleMap.put(1.4, 0.03);
        angleMap.put(2.3, 0.05);
        angleMap.put(3.0, 0.06);
        angleMap.put(4.0, 0.0625);
        angleMap.put(6.0, 0.065);

        upSpeedMap.put(0.0, 32.0);
        upSpeedMap.put(1.4, 32.0);
        upSpeedMap.put(2.3, 32.0);
        upSpeedMap.put(3.0, 32.0);
        upSpeedMap.put(4.0, 34.0);
        upSpeedMap.put(6.0, 34.0);

        downSpeedMap.put(0.0, 32.0);
        downSpeedMap.put(1.4, 40.0);
        downSpeedMap.put(2.3, 42.0);
        downSpeedMap.put(3.0, 46.0);
        downSpeedMap.put(4.0, 48.0);
        downSpeedMap.put(6.0, 51.0);
    }

    @Override
    public void execute() {
        if (doAimSupplier.getAsBoolean()) {
            Pose2d robotPose = robotPoseSupplier.get();
            double distance = robotPose.getTranslation().getDistance(AllianceFlipUtil.flip(kHubLocation));

            double angle = angleMap.get(distance);
            double upSpeed = upSpeedMap.get(distance);
            double downSpeed = downSpeedMap.get(distance);

            shooter.setAngle(angle);
            shooter.setSpeed(upSpeed, downSpeed);

            if (doShootSupplier.getAsBoolean()) {
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
