// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This sample program shows how to control a motor using a joystick. In the operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and motor controller inputs also range from -1 to 1
 * making it easy to work together.
 *
 * <p>In addition, the encoder value of an encoder connected to ports 0 and 1 is consistently sent
 * to the Dashboard.
 */
public class Robot extends TimedRobot {
  // KONSTANTS
  private static final int kMotor_1 = 10;
  private static final int kMotor_2 = 11;
  private static final int kMotor_3 = 12;
  private static final int kMotor_4 = 13;
  //private static final int kJoystickPort = 1;
  private static final int kXboxPort = 0;

  // MOTORS
  private final SparkMax m_motor1;
  private final SparkMax m_motor2;
  private final SparkMax m_motor3;
  private SparkMaxConfig motor1Config;
  private final SparkFlex m_motor4;
  private SparkFlexConfig motor4Config;
  // JOYSTICKS
  // private final Joystick m_joystick;
  private final XboxController m_xbox;

  /** Called once at the beginning of the robot program. */
  public Robot() {
    m_motor1 = new SparkMax(kMotor_1, MotorType.kBrushless);
    m_motor2 = new SparkMax(kMotor_2, MotorType.kBrushless);
    m_motor3 = new SparkMax(kMotor_3, MotorType.kBrushless);
    m_motor4 = new SparkFlex(kMotor_4, MotorType.kBrushless);

    motor1Config = new SparkMaxConfig();
    motor4Config = new SparkFlexConfig();

    motor1Config
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    motor1Config.encoder
        .positionConversionFactor(1000)
        .velocityConversionFactor(1000);
    // motor1Config.closedLoop
    //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //     .pid(1.0, 0.0, 0.0);
    
    motor4Config
        .inverted(true)
        .idleMode(IdleMode.kCoast);
    motor4Config.encoder
        .positionConversionFactor(1000)
        .velocityConversionFactor(1000);
    motor4Config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(1.0, 0.0, 0.0);
    
    m_motor1.configure(motor1Config, ResetMode.kResetSafeParameters,PersistMode.kNoPersistParameters);
    m_motor2.configure(motor1Config, ResetMode.kResetSafeParameters,PersistMode.kNoPersistParameters);
    m_motor3.configure(motor1Config, ResetMode.kResetSafeParameters,PersistMode.kNoPersistParameters);
    m_motor4.configure(motor4Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // m_joystick = new Joystick(kJoystickPort);
    m_xbox = new XboxController(kXboxPort);
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    // SmartDashboard.putNumber("Encoder", m_encoder.getDistance());
  }

  /** The teleop periodic function is called every control packet in teleop. */
  @Override
  public void teleopPeriodic() {
    if(m_xbox.getAButton()) {
      //Move motor1 in one direction
      m_motor1.set(0.4); 
      // m_xbox.setRumble(GenericHID.RumbleType.kRightRumble,0.5568968);
      // m_xbox.setRumble(GenericHID.RumbleType.kLeftRumble,0.5568968);
    }
    else if(m_xbox.getBButton()) {
      // Move motor1 in other direction
      m_motor1.set(-0.4); 
      // m_xbox.setRumble(GenericHID.RumbleType.kRightRumble,0.5568968);
      // m_xbox.setRumble(GenericHID.RumbleType.kLeftRumble,0.5568968);
    } 
    else {
      // Stop motor1
      m_motor1.set(0); 
      // m_xbox.setRumble(GenericHID.RumbleType.kRightRumble,0);
      // m_xbox.setRumble(GenericHID.RumbleType.kLeftRumble,0);
    }
      
    if(m_xbox.getXButton()) {
      //Move motor2 in one direction
      m_motor2.set(.4);
      // m_xbox.setRumble(GenericHID.RumbleType.kRightRumble,0.5568968);
      // m_xbox.setRumble(GenericHID.RumbleType.kLeftRumble,0.5568968);
    }
    else if(m_xbox.getYButton()) {
      // Move motor2 in other direction
      m_motor2.set(-0.4); 
      // m_xbox.setRumble(GenericHID.RumbleType.kRightRumble,0.5568968);
      // m_xbox.setRumble(GenericHID.RumbleType.kLeftRumble,0.5568968);
    }
    else {
      // Stop motor2
      m_motor2.set(0); 
      // m_xbox.setRumble(GenericHID.RumbleType.kRightRumble,0);
      // m_xbox.setRumble(GenericHID.RumbleType.kLeftRumble,0);
    }


  }
}
