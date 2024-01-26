package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePiston;


public class IntakeIn extends Command{
  // The subsystem the command runs on
  private final IntakePiston intake2;

  public IntakeIn(IntakePiston subsystem) {
    intake2 = subsystem;
    addRequirements(intake2);
  }

  @Override
  public void initialize() {
    intake2.intakeIn();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}