// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Commands.Boru_Run_Cmd;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SeviyeCommand extends SequentialCommandGroup {
  /** Creates a new SeviyeCommand. */
  SwerveSubsystem swerveSubsystem;
  int seviye;
  int position;
  public SeviyeCommand(Superstructure superstructure,int seviye) {
   if(seviye==0){position=0;}
   if(seviye==1){position=18;}
   if(seviye==2){position=35;}
   if(seviye==3){position=68;}
   if(seviye==4){position=117;}//122
   boolean boruvarmi=!(superstructure.getBoruSensor());
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
      addCommands(
        new Asansor_Pos_Cmd(superstructure, position),
        new Boru_Run_Cmd(superstructure,-0.8),
        new Asansor_Pos_Cmd(superstructure,0)
      ); 
    
 
     }
}