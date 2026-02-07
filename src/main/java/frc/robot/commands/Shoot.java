package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.util.AllianceFlipUtil;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class Shoot extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final InterpolatingDoubleTreeMap speedUpMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap speedDownMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
    public Shoot(
        ShooterSubsystem shooterSubsystem,
        Supplier<Pose2d> robotPoseSupplier
    ) {
        this.shooterSubsystem = shooterSubsystem;
        this.robotPoseSupplier = robotPoseSupplier;
        addRequirements(shooterSubsystem);

        speedUpMap.put(0.0, 0.0);//rpm
        speedUpMap.put(2.0, 1.0);//rpm
        speedDownMap.put(0.0, 0.0);//rpm
        speedDownMap.put(0.0, 0.0);//rpm
        angleMap.put(0.0, 0.0);
        angleMap.put(0.0, 0.0);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Pose2d currentPose = robotPoseSupplier.get();
        Translation2d goalPose = new Translation2d(2, 2);
        Translation2d trueTarget = AllianceFlipUtil.flip(goalPose);
        double distance = currentPose.getTranslation().getDistance(trueTarget);
        // some calculations here
        double shooterUpSpeed = speedUpMap.get(distance); // replace with actual calculation
        shooterSubsystem.setShooterUpSpeed(shooterUpSpeed);
        double ShooterDownSpeed = speedDownMap.get(distance);
        shooterSubsystem.setShooterDownSpeed(ShooterDownSpeed);
        double shooterAngleTarget = angleMap.get(distance);
        shooterSubsystem.setShooterTarget(shooterAngleTarget);       
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
