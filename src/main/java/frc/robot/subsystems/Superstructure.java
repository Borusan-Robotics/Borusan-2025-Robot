// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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
  private SparkMax motorLift;
  private SparkMax motorClimb;
  private SparkMax motorIntake;
  private SparkMaxConfig motorLiftConfig;
  private SparkMaxConfig motorClimbConfig;
  private SparkMaxConfig motorIntakeConfig;
  private RelativeEncoder encoderLift;
  private RelativeEncoder encoderClimb;
  private DigitalInput sensorIntake;
  private DigitalInput angleSwitch;
  public static final double kG = 0.0015; // Yerçekimi için besleme ileri
  private Double lastTarget=null;
  private static final double POSITION_DEADBAND = 1.5;
  

  public Superstructure() {
    sensorIntake=new DigitalInput(0);
    angleSwitch=new DigitalInput(2);
    motorIntake=new SparkMax(23, MotorType.kBrushless);
    motorIntakeConfig= new SparkMaxConfig();
    motorIntakeConfig.idleMode(IdleMode.kBrake);
    motorIntake.configure(motorIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    CameraServer.startAutomaticCapture();
    CvSink cvSink=CameraServer.getVideo();
    CvSource outputStream= CameraServer.putVideo("cam", 640, 480);


    /////////////////////// ASANSÖR BAŞ  ///////////////////////////////////////////////
    motorLift = new SparkMax(21, MotorType.kBrushless);
    encoderLift = motorLift.getEncoder();
    motorLiftConfig = new SparkMaxConfig();
    motorLiftConfig.smartCurrentLimit(60);
    motorLiftConfig.encoder
        .positionConversionFactor(1);
    motorLiftConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.40)  // Düşürüldü: Daha yumuşak yaklaşım
        .i(0)  // Eklendi: Steady-state hatayı düzeltir
        .d(0.032)  // Eklendi: Salınımı önler ve yumuşatır
        .outputRange(-1, 1);
    motorLiftConfig.closedLoop.maxMotion
      .maxVelocity(8000 )  // Düşürüldü: Daha kontrollü
      .maxAcceleration(6000)  // Düşürüldü: Daha yumuşak
      .allowedClosedLoopError(2);  // Artırıldı: Daha toleranslı
     encoderLift.setPosition(0);
    motorLift.configure(motorLiftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    /////////////////////// ASANSÖR BİTİŞ  ///////////////////////////////////////////////


  ///////////////////////     TIRMANMA BAŞ    ////////////////////////
    motorClimb=new SparkMax(22, MotorType.kBrushless);
    encoderClimb=motorClimb.getEncoder();
    motorClimbConfig=new SparkMaxConfig();
    motorClimbConfig.encoder.positionConversionFactor(1);
    motorClimbConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.4)
    .i(0)
    .d(0.0005)
    .outputRange(-1, 1);

    motorClimbConfig.closedLoop.maxMotion
      .maxVelocity(2000)
      .maxAcceleration(2500)
      .allowedClosedLoopError(1);
    
    encoderClimb.setPosition(0);
    motorClimb.configure(motorClimbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  ///////////////////////     TIRMANMA BİT    ////////////////////////
    
  

  }
  public void setLimelightPipline(){LimelightHelpers.getCurrentPipelineIndex("pipline");} 
  public boolean getLimeLightTargetValid(){return LimelightHelpers.getTV("limelight");}
  public double getLimeLightTargetOffsetx(){return LimelightHelpers.getTX("tx");}
  public double getLimeLightTargetOffsety(){return LimelightHelpers.getTY("ty");} 
  public void liftPos(double pos) {
    double feedforward = kG;
    // Only update if target is far enough from the last one
    if (lastTarget != null && Math.abs(pos - lastTarget) < POSITION_DEADBAND) {
        return; // Skip reissuing nearly identical reference
    }

    lastTarget = pos; // remember this target

    motorLift.getClosedLoopController().setReference(
        pos,
        ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0,
        feedforward // feedforward
    );
  
}
  public void liftRun(double spd){motorLift.set(spd);}
  public void liftStop(){motorLift.stopMotor();}
  public double getLiftEncoder(){return encoderLift.getPosition();}

  public boolean getAngleSwitch(){return angleSwitch.get();}
  public void climbPos(double pos){motorClimb.getClosedLoopController().setReference(pos, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);}
  public void climbRun(double spd){motorClimb.set(spd);}
  public void climbStop(){motorClimb.stopMotor();}
  public double getClimbEncoder(){return encoderClimb.getPosition();}
  public void IntakeRun(double spd){motorIntake.set(spd);}
  public void IntakeStop(){motorIntake.stopMotor();}
  public boolean getObjectSensor(){return sensorIntake.get();}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Asansor Pos", encoderLift.getPosition());
    SmartDashboard.putNumber("Tirmanma Pos", encoderClimb.getPosition());
    SmartDashboard.putBoolean("Boru Sensor", !getObjectSensor());
    SmartDashboard.putBoolean("LimitSwitch",getAngleSwitch());
  }
}