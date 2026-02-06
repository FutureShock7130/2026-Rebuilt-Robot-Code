package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class index extends Command {
    private final IndexerSubsystem indexerSubsystem;
    private final double indexerUpSpeed;
    private final double indexerDownSpeed;
    public index(IndexerSubsystem indexerSubsystem, double indexerUpSpeed, double indexerDownSpeed) {
        this.indexerSubsystem = indexerSubsystem;
        this.indexerUpSpeed = indexerUpSpeed;
        this.indexerDownSpeed = indexerDownSpeed;
        addRequirements(indexerSubsystem);
    }
    @Override
    public void initialize() {
        indexerSubsystem.setIndexerUpSpeed(indexerUpSpeed);
        indexerSubsystem.setIndexerDownSpeed(indexerDownSpeed);
    }
    @Override
    public void execute() {
    }
    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stop();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
