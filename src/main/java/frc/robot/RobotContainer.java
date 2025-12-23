// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;


import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.Asansor_Pos_Cmd;
import frc.robot.Commands.Asansor_Run_Cmd;
import frc.robot.Commands.SeviyeCommand;

import frc.robot.Commands.Tirmanma_Run_Cmd;
import frc.robot.Commands.Boru_Run_Cmd;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static Superstructure m_Super = new Superstructure();
  public static SwerveSubsystem drivebase= new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  final XboxController m_xbox_1=new XboxController(0);
  final XboxController m_xbox_2=new XboxController(2);
  final Joystick MdriveStation=new Joystick(1);

  public RobotContainer() {
    // Configure the trigger bindings
    NamedCommands.registerCommand("Seviye 4",new SeviyeCommand(m_Super, 4));
    NamedCommands.registerCommand("BoruAl",new Boru_Run_Cmd(m_Super, -0.75));
    configureBindings();
  }

  private void configureBindings() {
    ////////////////  COMMAND BAŞ ///////////////////////// 
      new JoystickButton(m_xbox_1, 4).whileTrue(new Boru_Run_Cmd(m_Super, 0.6));
      new JoystickButton(m_xbox_1, 1).whileTrue(new Boru_Run_Cmd(m_Super, -0.6)); 
      new JoystickButton(MdriveStation, 5).whileTrue(new Tirmanma_Run_Cmd(m_Super, 0.7));//geri al
      new JoystickButton(MdriveStation, 10).whileTrue(new Tirmanma_Run_Cmd(m_Super, -0.7));//aç   
      new JoystickButton(MdriveStation, 6).onTrue(new SeviyeCommand(m_Super, 1));
      new JoystickButton(MdriveStation, 12).onTrue(new SeviyeCommand(m_Super, 2));
      new JoystickButton(MdriveStation, 7).onTrue(new SeviyeCommand(m_Super, 3));
      new JoystickButton(MdriveStation, 9).onTrue(new SeviyeCommand(m_Super, 4));
      new JoystickButton(MdriveStation, 11).onTrue(new Asansor_Pos_Cmd(m_Super, 0));
      new JoystickButton(m_xbox_2, 1).whileTrue(new Asansor_Run_Cmd(m_Super, -1));            
      new JoystickButton(m_xbox_2, 4).whileTrue(new Asansor_Run_Cmd(m_Super, 1));            
    ////////////////  COMMAND BİTİŞ //////////////////////
  }
    
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");  }

 /*public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  } */ 
}
