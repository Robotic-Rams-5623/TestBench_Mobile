// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



// import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
// import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

  private double percent = 0;

  private boolean prevDUp = false;
  private boolean prevDDown = false;
  
  // KONSTANTS
  // private static final int kMotor_1 = 11;
  // private static final int kMotor_2 = 12;
  // private static final int kMotor_3 = 13;
  private static final int kMotor_4 = 1;
  private static final int kMotor_5 = 2;
  //private static final int kJoystickPort = 1;
  private static final int kXboxPort = 0;

  // DIGITAL INPUTS
  private final DigitalInput m_di0;


  // MOTORS
  // private final SparkMax m_motor1;
  // private final SparkMax m_motor2;
  // private final SparkMax m_motor3;
  private final SparkFlex m_motor4;
  private final SparkFlex m_motor5;
  // private SparkMaxConfig motor1Config;
  // private SparkMaxConfig motor2Config;
  // private SparkFlexConfig motor3Config;
  private SparkFlexConfig flexmotorConfig;
  // private SparkFlexConfig flexmotorConfig1;
  // private RelativeEncoder flywheelEncoder;



  // JOYSTICKS
  // private final Joystick m_joystick;
final CommandXboxController m_xbox;
private final Trigger XboxA;
private final Trigger XboxB;
@SuppressWarnings("unused")
private final Trigger XboxX;
private final Trigger XboxY;
private final Trigger XboxDpadDown;
private final Trigger XboxDpadUp;


  /** Called once at the beginning of the robot program. */
  public Robot() {
    // m_motor1 = new SparkMax(kMotor_1, MotorType.kBrushless);
    // m_motor2 = new SparkMax(kMotor_2, MotorType.kBrushless);
    // m_motor3 = new SparkMax(kMotor_3, MotorType.kBrushless);
    m_motor4 = new SparkFlex(kMotor_4, MotorType.kBrushless);
    m_motor5 = new SparkFlex(kMotor_5, MotorType.kBrushless);

    // motor1Config = new SparkMaxConfig();
    // motor2Config = new SparkMaxConfig();
    // motor3Config = new SparkFlexConfig();
    flexmotorConfig = new SparkFlexConfig();
        
    m_xbox = new CommandXboxController(kXboxPort);

    XboxA = m_xbox.a();
    XboxB = m_xbox.b();
    XboxX = m_xbox.x();
    XboxY = m_xbox.y();
    XboxDpadDown = m_xbox.povDown();
    XboxDpadUp = m_xbox.povUp();

    // flywheelEncoder = m_motor4.getEncoder();

    // motor1Config
    //     .inverted(false)
    //     .idleMode(IdleMode.kCoast);

         
    
    // motor2Config
    //   .inverted(false)
    //   .idleMode(IdleMode.kCoast);
       
    // motor3Config
    //    .inverted(false)
    //    .idleMode(IdleMode.kCoast);

    
    flexmotorConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast);

    
    // m_motor1.configure(motor1Config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    // m_motor2.configure(motor2Config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    // m_motor3.configure(motor3Config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    m_motor4.configure(flexmotorConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    m_motor5.configure(flexmotorConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    // m_joystick = new Joystick(kJoystickPort);
    m_di0 = new DigitalInput(0);
    
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("Digital Input", m_di0.get());
    // SmartDashboard.putNumber("Encoder", m_encoder.getDistance());
  }

  /** The teleop periodic function is called every control packet in teleop. */
  
  
  @Override
public void teleopPeriodic() {
  if (XboxA.getAsBoolean()) {
    m_motor4.set(-percent);
    m_motor5.set(percent);
  } 

  else {
    m_motor4.set(0);
    m_motor5.set(0);
  }

  if (XboxDpadDown.getAsBoolean() && !prevDDown && percent > -1) {
    percent -= 0.05;
  }

  if (XboxDpadUp.getAsBoolean() && !prevDUp && percent < 1) {
    percent += 0.05;
  }

  prevDDown = XboxDpadDown.getAsBoolean();
  prevDUp = XboxDpadUp.getAsBoolean();
  SmartDashboard.putNumber("Launcher Percentage", percent);
}
}