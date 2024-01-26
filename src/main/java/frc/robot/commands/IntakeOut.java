package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePiston;


public class IntakeOut extends Command{
  // The subsystem the command runs on
  private final IntakePiston intake;

  public IntakeOut(IntakePiston subsystem) {
    intake = subsystem;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.intakeOut();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}