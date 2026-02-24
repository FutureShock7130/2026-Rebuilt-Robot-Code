package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final double intakeSP;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, double intakeSP) {
        this.intakeSubsystem = intakeSubsystem;
        this.intakeSP = intakeSP;
   }

   public void execute(){
        intakeSubsystem.runIntake(intakeSP);
   }
}
