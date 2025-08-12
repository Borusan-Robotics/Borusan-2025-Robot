// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OperatorConstants;
import swervelib.SwerveInputStream;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static Robot instance;
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  final XboxController driverXbox = new XboxController(0);

  public boolean loadMode = false;
  public boolean mainMode = false;
  public static double ChaseAngle; 
  double idnumber;
  
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    String limelightURL = "http://10.49.72.200:5801/stream.mjpg";
    Shuffleboard.getTab("cam").add("limelihgt",limelightURL)
    .withWidget(BuiltInWidgets.kCameraStream)
    .withProperties(Map.of("Show Crosshair",true,"Show Controls",false));
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }
  

 
  double xSpeed = 0.0;
  double ySpeed = 0.0;
  double zSpeed = 0.0;
  boolean robotMerkezliMi = false;
  //double zKp = 0.026;
  //double zKi = 0.035;
  //double zKd = 0.00012;
  //double zKd = 0.000053;
  //double zKi = 0.04;
  double minOutput = 0.15;
  double rampingfactor = 0.65;
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (driverXbox.getStartButton()) {
      RobotContainer.drivebase.zeroGyro();
      
    }
    if (driverXbox.getXButton()) {
      RobotContainer.drivebase.lock();
    }

//////////////     LİMELİGHT BAŞ     /////////////////////////////////

    double tx = LimelightHelpers.getTX("limelight");
    double ty = LimelightHelpers.getTY("limelight");
    boolean tv = LimelightHelpers.getTV("limelight");  
    //SmartDashboard.putNumber("aprilid",idnumber);  
    SmartDashboard.putNumber("ty pos", ty);
    SmartDashboard.putNumber("tx pos", tx);
    SmartDashboard.getBoolean("tv", tv);  
    PIDController SagxPID = new PIDController(0.0300,0, 0);
    PIDController SagyPID = new PIDController(0.0163, 0.0, 0);
    PIDController SolxPID = new PIDController(0.026,0, 0);
    PIDController SolyPID = new PIDController(0.0158, 0, 0);
    PIDController zPID = new PIDController(0.0255, 0.04, 0.0000535);    
    idnumber = LimelightHelpers.getLimelightNTDouble("limelight", "tid");
    SmartDashboard.putNumber("AprilTag ID", idnumber);
    updateChaseAngle(idnumber); 
    SmartDashboard.putNumber("navx yaw", RobotContainer.drivebase.yawvalue());
   
    
    
    
    //// LIMELİGHT ORJ KOD BAS /////
      
    if(driverXbox.getRawButton(2)){
      LimelightHelpers.setPipelineIndex("resif", 1);
      double batteryvoltage = RobotController.getBatteryVoltage();
      double nominalVoltage = 12.4;
      double voltageCompensation = 1.0;//nominalVoltage/batteryvoltage;           
      if(tv){           
            zSpeed= zPID.calculate(RobotContainer.drivebase.getHeading().getDegrees(),getChaseAngle()) *-voltageCompensation;
           
            
            if(Math.abs(zSpeed) > 0.65){
              zSpeed = Math.copySign(0.65, zSpeed);
            }
            //System.out.println(zSpeed); 
            System.out.println(getChaseAngle());
      }
      else{
        zSpeed = 0.0;
      }   
    }  
    else if (driverXbox.getRightBumperButton()){
          LimelightHelpers.setPipelineIndex("resif", 1);
          double batteryvoltage = RobotController.getBatteryVoltage();
          double nominalVoltage = 12.4;
          double voltageCompensation =1.0;// nominalVoltage/batteryvoltage;          
          if(tv == true){
            robotMerkezliMi = true;
            xSpeed = SagxPID.calculate(tx,0.105)* -voltageCompensation;
            if(Math.abs(xSpeed) > 0.55){ 
              xSpeed = Math.copySign(0.55, xSpeed);
            }

            System.out.println("X "+xSpeed);


            ySpeed = SagyPID.calculate(ty, 0.2) * -voltageCompensation;
            if(Math.abs(ySpeed) > 0.55){
              ySpeed = Math.copySign(0.55, ySpeed);
            } 
            System.out.println("Y "+ySpeed);
          }          
        else{
          xSpeed = 0.0;
          ySpeed = 0.0;          
        }
    }else if (driverXbox.getLeftBumperButton()){
      LimelightHelpers.setPipelineIndex("resif", 1);
      double batteryvoltage = RobotController.getBatteryVoltage();
      double nominalVoltage = 12.4;
      double voltageCompensation = 1.0;//nominalVoltage/batteryvoltage;          
      if(tv == true){
        robotMerkezliMi = true;
        xSpeed = SolxPID.calculate(tx,30.0)* -voltageCompensation;
        if(Math.abs(xSpeed) > 0.3){ 
          xSpeed = Math.copySign(0.3, xSpeed);
        }

        System.out.println(xSpeed);


        ySpeed = SolyPID.calculate(ty, 0.0) * -voltageCompensation;
        if(Math.abs(ySpeed) > rampingfactor){
          ySpeed = Math.copySign(rampingfactor, ySpeed);
        } 
        //System.out.println(ySpeed);
      }          
    else{
      xSpeed = 0.0;
      ySpeed = 0.0;          
    }
}
    
    else {
      robotMerkezliMi=false;
      xSpeed = driverXbox.getLeftX() * -0.8;
      ySpeed = driverXbox.getLeftY() * -0.8;
      zSpeed = driverXbox.getRightX() * 0.8;
    }
    
     //// LIMELİGHT ORJ KOD BIT /////


    SmartDashboard.putBoolean("field relative", robotMerkezliMi);
    
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(RobotContainer.drivebase.getSwerveDrive(),
        () -> ySpeed,
        () -> xSpeed)
        .withControllerRotationAxis(() -> zSpeed)
        .robotRelative(()->robotMerkezliMi)
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(()->!robotMerkezliMi);
      

    Command driveFieldOrientedAnglularVelocity = RobotContainer.drivebase.driveFieldOriented(driveAngularVelocity);
  
    
    RobotContainer.drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  
    
  }
  public void updateChaseAngle(double idnumber) {
    if (idnumber == 10 || idnumber == 21) {
        ChaseAngle = 180;
    } else if (idnumber == 7 || idnumber == 18) {
        ChaseAngle = 0;
    } else if (idnumber == 9 || idnumber == 22) {
        ChaseAngle = 120;
    } else if (idnumber == 6 || idnumber == 19) {
        ChaseAngle = -60;
    } else if (idnumber == 8 || idnumber == 17) {
        ChaseAngle = 60;
    } else if (idnumber == 11 || idnumber == 20) {
        ChaseAngle = -120;
    }
}

  public static double getChaseAngle() {
    return ChaseAngle;
  }
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
