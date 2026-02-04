package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimberCommand extends Command{
   private final ClimberSubsystem climberSubsystem;
   private final double climberPosition;

   public ClimberCommand(ClimberSubsystem climberSubsystem, double climberPosition) {
       this.climberSubsystem = climberSubsystem;
       this.climberPosition = climberPosition;
   }
   public void initialize() {
     
        climberSubsystem.setClimberTargetPosition(climberPosition);
   }
    public void execute() {
          climberSubsystem.runClimber();
          SmartDashboard.putNumber("climberMotor", climberSubsystem.getClimberPosition());
          SmartDashboard.putNumber("climberTarget", climberPosition);
    }
    public void end(boolean interrupted) {
          climberSubsystem.resetClimberPosition();
    }
    public boolean isFinished() {
          return false;
    }
}
