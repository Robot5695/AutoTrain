// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  //private DifferentialDrive m_myRobot;
 // private Joystick m_leftStick;
  //private Joystick m_rightStick;

  //private final MotorController m_leftMotor = new PWMSparkMax(0);
  //private final MotorController m_rightMotor = new PWMSparkMax(1);
XboxController driver1;
CANSparkMax left1=new CANSparkMax(1, MotorType.kBrushless);
CANSparkMax left2=new CANSparkMax(2, MotorType.kBrushless);
CANSparkMax right1=new CANSparkMax(3, MotorType.kBrushless);
CANSparkMax right2=new CANSparkMax(4, MotorType.kBrushless);

RelativeEncoder left1_encoder = left1.getEncoder();
RelativeEncoder right1_encoder = right1.getEncoder();

Timer m_timer = new Timer();

double Kspeed;
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //m_rightMotor.setInverted(true);
    right1.setInverted(true);
    right2.setInverted(true);
right2.follow(right1);
left2.follow(left1);

    //m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
   // m_leftStick = new Joystick(0);
   //m_rightStick = new Joystick(1);
   driver1=new XboxController(0);
  }

  @Override
  public void teleopPeriodic() {
   // m_myRobot.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());

left1.set(-driver1.getLeftY()+driver1.getRightX());
right1.set(-driver1.getLeftY()-driver1.getRightX());
  }

  
  

/** This function is run once each time the robot enters autonomous mode. */
@Override
public void autonomousInit() {

  m_timer.reset();
  m_timer.start();
}

/** This function is called periodically during autonomous. */
@Override
public void autonomousPeriodic() {
  SmartDashboard.putNumber("Left Drive Encoder", left1_encoder.getPosition());
    SmartDashboard.putNumber("Right Drive Encoder", right1_encoder.getPosition());
  SmartDashboard.putNumber("Timer:",m_timer.get());
  

  // Drive for 2 seconds
  if (m_timer.get() <1.5) {
    Kspeed=0.2;
  } else {
    Kspeed=0;
  }


  right1.set(Kspeed);
  left1.set(Kspeed);
  
}












}
