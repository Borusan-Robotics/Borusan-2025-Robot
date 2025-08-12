// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Asansor_Pos_Cmd extends Command {
      private final Superstructure superstructure;
    double position;
  /** Creates a new Asansor_Pos_Cmd. */
  public Asansor_Pos_Cmd(Superstructure m_super,double pos) {
    superstructure=m_super;
    position=pos;
    addRequirements(m_super);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    superstructure.AsansorPos(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(superstructure.GetAsansorEncoder()>=(position-2) && superstructure.GetAsansorEncoder()<=(position+2)){
    return true;}
    else{
      return false;
    }
  }
}
