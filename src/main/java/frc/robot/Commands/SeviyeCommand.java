package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Superstructure;

public class SeviyeCommand extends SequentialCommandGroup {
  
  // Constants for positions
  private static final int GROUND_LEVEL = 0;
  private static final int LEVEL_1 = 10;
  private static final int LEVEL_2 = 18;
  private static final int LEVEL_3 = 40;
  private static final int LEVEL_4 = 71;
  
  // Constants for speeds
  private static final double INTAKE_SPEED = -0.8;

  public SeviyeCommand(Superstructure superstructure, int seviye) {
    int position = getPositionForLevel(seviye);
    
    addCommands(
      new Asansor_Pos_Cmd(superstructure, position),
      new Boru_Run_Cmd(superstructure, INTAKE_SPEED),
      new Asansor_Pos_Cmd(superstructure, GROUND_LEVEL)
    ); 
  }
  
  private int getPositionForLevel(int seviye) {
    switch (seviye) {
      case 0: return GROUND_LEVEL;
      case 1: return LEVEL_1;
      case 2: return LEVEL_2;
      case 3: return LEVEL_3;
      case 4: return LEVEL_4;
      default: 
        System.err.println("Invalid level: " + seviye + ". Using ground level.");
        return GROUND_LEVEL;
    }
  }
}