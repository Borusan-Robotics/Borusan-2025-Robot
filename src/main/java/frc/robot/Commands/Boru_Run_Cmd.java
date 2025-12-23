// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Boru_Run_Cmd extends Command {
  Superstructure superstructure;
  Double speed;
  Boolean isObjectThere=false;
  /** Creates a new Boru_Run_Cmd. */
  public Boru_Run_Cmd(Superstructure m_super,double spd) {
    this.superstructure=m_super;
    this.speed=spd*-1;    
    addRequirements(m_super);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  isObjectThere=!(superstructure.getObjectSensor());
  }

  boolean isFinished=false;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(isObjectThere)
    {
      if(!(superstructure.getObjectSensor())){
        superstructure.IntakeRun(speed);
        isFinished=false;
      }
      else{
        superstructure.IntakeRun(0.0);
        isFinished=true;
      }
    }else{
      if(!(superstructure.getObjectSensor())){
        superstructure.IntakeRun(0);
        isFinished=true;
      }
      else{
        superstructure.IntakeRun(speed);
        isFinished=false;
      }  
    }
    

    
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    superstructure.IntakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
