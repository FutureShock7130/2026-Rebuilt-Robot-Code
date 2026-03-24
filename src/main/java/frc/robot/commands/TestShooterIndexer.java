package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.kHubLocation;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.util.AllianceFlipUtil;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class TestShooterIndexer extends Command {
    private final Shooter shooter;
    private final Indexer indexer;

    private final XboxController driverJoystick;

    private final Supplier<Pose2d> robotPoseSupplier;

    private boolean recordResult = false;
    private String testResult = "";
    
    public TestShooterIndexer(Shooter shooter, Indexer indexer, XboxController joystick, Supplier<Pose2d> robotPoseSupplier) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.driverJoystick = joystick;
        this.robotPoseSupplier = robotPoseSupplier;

        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("TestShooter/Angle", 0.0);
        SmartDashboard.putNumber("TestShooter/UpSpeed", 0.0);
        SmartDashboard.putNumber("TestShooter/DownSpeed", 0.0);
    }

    @Override
    public void execute() {
        shooter.setAngle(SmartDashboard.getNumber("TestShooter/Angle", 0.0));
        if (driverJoystick.getRightBumperButton()) {
            shooter.setSpeed(SmartDashboard.getNumber("TestShooter/UpSpeed", 0.0), SmartDashboard.getNumber("TestShooter/DownSpeed", 0.0));
        } else {
            shooter.setSpeed(0.0, 0.0);
        }
        if (driverJoystick.getRightTriggerAxis() > 0.5) {
            indexer.set(IndexerConstants.kIndexingSpeed, IndexerConstants.kIndexingSpeed);
        } else {
            indexer.set(0, 0);
        }

        if (!recordResult && SmartDashboard.getBoolean("TestShooter/Record", false)) {
            Pose2d robotPose = robotPoseSupplier.get();
            double distance = robotPose.getTranslation().getDistance(AllianceFlipUtil.flip(kHubLocation));
            recordResult = true;
            testResult += String.format(
                "angleMap.put(%.2f, %.2f);\nupSpeedMap.put(%.2f, %.2f);\ndownSpeedMap.put(%.2f, %.2f);\n\n",
                distance,
                SmartDashboard.getNumber("TestShooter/Angle", 0.0),
                distance,
                SmartDashboard.getNumber("TestShooter/UpSpeed", 0.0),
                distance,
                SmartDashboard.getNumber("TestShooter/DownSpeed", 0.0)
            );
            SmartDashboard.putString("TestShooter/Result", testResult);
        }

        recordResult = SmartDashboard.getBoolean("TestShooter/Record", false);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setAngle(0.0);
        shooter.setSpeed(0.0, 0.0);
        indexer.set(0.0, 0.0);
        System.out.println("TestShooterIndexer ended, result:\n" + testResult);
    }
}
