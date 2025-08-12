// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Superstructure extends SubsystemBase {
  /** Creates a new Superstructure. */
  private SparkMax motorAsansor;
  private SparkMax motorTirmanma;
  private SparkMax motorIntake;
  private SparkMaxConfig motorAsansorConfig;
  private SparkMaxConfig motorTirmanmaConfig;
  private SparkMaxConfig motorIntakeConfig;
  private SparkClosedLoopController closedLoopController;
  private SparkClosedLoopController closedLoopController3;
  private RelativeEncoder encoderAsansor;
  private RelativeEncoder encoderTirmanma;
  private DigitalInput sensorIntake;
  private DigitalInput aciSwitch;
  LimelightHelpers limelight;
  double kG=0.010;
  


  public Superstructure() {
    sensorIntake=new DigitalInput(0);
    aciSwitch=new DigitalInput(2);
    motorIntake=new SparkMax(23, MotorType.kBrushless);
    motorIntakeConfig= new SparkMaxConfig();
    motorIntakeConfig.idleMode(IdleMode.kBrake);
    motorIntake.configure(motorIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    CameraServer.startAutomaticCapture();
    CvSink cvSink=CameraServer.getVideo();
    CvSource outputStream= CameraServer.putVideo("cam", 640, 480);




    /////////////////////// ASANSÖR BAŞ  ///////////////////////////////////////////////
    motorAsansor = new SparkMax(21, MotorType.kBrushless);
    encoderAsansor = motorAsansor.getEncoder();
    motorAsansorConfig = new SparkMaxConfig();
    closedLoopController = motorAsansor.getClosedLoopController();
    motorAsansorConfig.encoder
        .positionConversionFactor(1);
    motorAsansorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(1)
        .i(0)
        .d(0)
        .outputRange(-1, 1);
    motorAsansorConfig.closedLoop.maxMotion
      // Set MAXMotion parameters for position control. We don't need to pass
      // a closed loop slot, as it will <default to slot 0.
      .maxVelocity(8000)
      .maxAcceleration(8000)
      .allowedClosedLoopError(1);
     encoderAsansor.setPosition(0);
    motorAsansor.configure(motorAsansorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    /////////////////////// ASANSÖR BİTİŞ  ///////////////////////////////////////////////

    

  ///////////////////////     TIRMANMA BAŞ    ////////////////////////

    motorTirmanma=new SparkMax(22, MotorType.kBrushless);
    
    closedLoopController3=motorTirmanma.getClosedLoopController();
    encoderTirmanma=motorTirmanma.getEncoder();

  
    motorTirmanmaConfig=new SparkMaxConfig();

    motorTirmanmaConfig.encoder
        .positionConversionFactor(1);
        
  
    motorTirmanmaConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.4)
    .i(0)
    .d(0.0005
    )
    .outputRange(-1, 1);

    motorTirmanmaConfig.closedLoop.maxMotion
        .maxVelocity(2000)
        .maxAcceleration(2500)
        .allowedClosedLoopError(1);
    
    encoderTirmanma.setPosition(0);
    motorTirmanma.configure(motorTirmanmaConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  ///////////////////////     TIRMANMA BİT    ////////////////////////


    limelight=new LimelightHelpers();
  }
  public void setLimelihgtPipline(){
    limelight.getCurrentPipelineIndex("pipline ");
  } 
  public boolean getLimeLihgitTargetValid(){
    return limelight.getTV("limelight");
  }
  
  public double getLimeLihgitTargetOffsetx(){
    return limelight.getTX("tx");
  }
  public  double getLimeLihgitTargetOffsety(){
    return limelight.getTY("ty");
  } 
  
  public void AsansorPos(double pos){
    double feedforward=kG;
    closedLoopController.setReference(pos, ControlType.kMAXMotionPositionControl,ClosedLoopSlot.kSlot0,feedforward);
  }
  public void AsansorRun(double spd){
    motorAsansor.set(spd);
  }
  public double GetAsansorEncoder(){
    return encoderAsansor.getPosition();
  }
  public void AsansorStop(){
    motorAsansor.stopMotor();
  }

  public boolean getAciSwitch(){
    return aciSwitch.get();
  }

  public void TirmanmaPos(double pos){
    closedLoopController3.setReference(pos, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }
  public void TirmanmaRun(double spd){
    motorTirmanma.set(spd);
  }
  public void TirmanmaStop(){
    motorTirmanma.stopMotor();
  }
  public double GetTirmanmaEncoder(){
    return encoderTirmanma.getPosition();
  }

  public void IntakeRun(double spd){
    motorIntake.set(spd);
  }
  public void IntakeStop(){
    motorIntake.stopMotor();
  }
  public boolean getBoruSensor(){
    return sensorIntake.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Asansor Pos", encoderAsansor.getPosition());
    SmartDashboard.putNumber("Tirmanma Pos", encoderTirmanma.getPosition());
    SmartDashboard.putBoolean("Boru Sensor", !getBoruSensor());
    SmartDashboard.putBoolean("LimitSwitch",getAciSwitch());
  }
}
