package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class Intake extends Command {
  private Manipulator m_manipulator;

  public Intake(Manipulator manipulator) {
    this.m_manipulator = manipulator;
    addRequirements(m_manipulator);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    m_manipulator.spinIntakeMotor();
  }

  @Override
  public void end(boolean interrupted) {
  m_manipulator.stopShooterMotor();
  }

  @Override
  public boolean isFinished() {
    if (m_manipulator.algaeChecker()==1){
      return true; 
    } else {
      return false;
    }
  }
}
