// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
XboxController driver1, driver2;
CANSparkMax left1=new CANSparkMax(1, MotorType.kBrushless);
CANSparkMax left2=new CANSparkMax(2, MotorType.kBrushless);
CANSparkMax right1=new CANSparkMax(3, MotorType.kBrushless);
CANSparkMax right2=new CANSparkMax(4, MotorType.kBrushless);
CANSparkMax arm1; 
CANSparkMax extender;
RelativeEncoder armEncoder;
RelativeEncoder extenderEncoder;
 private ADIS16470_IMU imu = new ADIS16470_IMU();


RelativeEncoder left1_encoder = left1.getEncoder();
RelativeEncoder right1_encoder = right1.getEncoder();

Timer m_timer = new Timer();
private static final String moveauto = "moveauto";
private static final String parkramp = "parkramp";
private String m_autoSelected;
private final SendableChooser<String> m_chooser = new SendableChooser<>();
private SparkMaxPIDController m_pidController;
private SparkMaxPIDController m_pidExtender;
public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, rotations, extenderrotations;


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

imu.calibrate();
m_chooser.setDefaultOption("parkramp", parkramp);
m_chooser.addOption("moveauto", moveauto);
SmartDashboard.putData("Auto choices", m_chooser);
    //m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
   // m_leftStick = new Joystick(0);
   //m_rightStick = new Joystick(1);
   driver1=new XboxController(0);
   driver2=new XboxController(1);

   arm1=new CANSparkMax(5, MotorType.kBrushless);
   extender=new CANSparkMax(6, MotorType.kBrushless);
   extender.setInverted(true);
   armEncoder=arm1.getEncoder();
   extenderEncoder=extender.getEncoder();
   m_pidController=arm1.getPIDController();
   m_pidExtender=extender.getPIDController();
   kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    m_pidExtender.setP(kP);
    m_pidExtender.setI(kI);
    m_pidExtender.setD(kD);
    m_pidExtender.setIZone(kIz);
    m_pidExtender.setFF(kFF);
    m_pidExtender.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
  }


  @Override
  public void teleopPeriodic() {
   // m_myRobot.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());

   double x = driver1.getRightX();
   double y = -driver1.getLeftY();
 if (x < 0.25 && x > -0.25) {
  x = 0;
 }
 if (y < 0.25 && y > -0.25) {
  y = 0;
 }
left1.set(y + x);
right1.set(y - x);

SmartDashboard.putNumber("Left Drive Encoder", left1_encoder.getPosition());
SmartDashboard.putNumber("Right Drive Encoder", right1_encoder.getPosition());
SmartDashboard.putNumber("Timer:",m_timer.get());
SmartDashboard.putNumber("angle", imu.getAngle());
SmartDashboard.putNumber("X", imu.getAccelX());
SmartDashboard.putNumber("Y", imu.getAccelY());
SmartDashboard.putNumber("Z", imu.getAccelZ());
  }

  
  

/** This function is run once each time the robot enters autonomous mode. */
@Override
public void autonomousInit() {

  m_timer.reset();
  m_timer.start();
  m_autoSelected = m_chooser.getSelected();
  System.out.println("Auto selected:" + m_autoSelected);
  left1_encoder.setPosition(0);
  right1_encoder.setPosition(0);
}

/** This function is called periodically during autonomous. */
@Override
public void autonomousPeriodic() {
 switch (m_autoSelected) {
  case moveauto:
  // Drive for 2 seconds
  if (m_timer.get() <1.5) {
    Kspeed=0.2;
  } else {
    Kspeed=0;
  }


  right1.set(Kspeed);
  left1.set(Kspeed);
  break;
  case parkramp:
int parkrampstate = 0;
double slow_speed = 0.05;
right1.set(slow_speed);
left1.set(slow_speed); 
System.out.println ("here");
//distance to top of hill is ~ 38
while (left1_encoder.getPosition()<38) {
//do nothing
SmartDashboard.putNumber("Y", imu.getAccelY());
SmartDashboard.putNumber("Left Drive Encoder", left1_encoder.getPosition());
SmartDashboard.putNumber("Right Drive Encoder", right1_encoder.getPosition());
}
while (true) {
  SmartDashboard.putNumber("Y", imu.getAccelY());
  SmartDashboard.putNumber("Left Drive Encoder", left1_encoder.getPosition());
  SmartDashboard.putNumber("Right Drive Encoder", right1_encoder.getPosition());
if (imu.getAccelY()<-2){
//Uphill 
right1.set(slow_speed);
left1.set(slow_speed);

}
else if (imu.getAccelY()>2){
//downhill
right1.set(-slow_speed);
left1.set(-slow_speed);

}
else {
//flat
right1.set(0);
left1.set(0);
right1.setIdleMode(IdleMode.kBrake);
right2.setIdleMode(IdleMode.kBrake);
left1.setIdleMode(IdleMode.kBrake);
left2.setIdleMode(IdleMode.kBrake);
}

}

  //break;
}
}

@Override 
public void testInit (){

armEncoder.setPosition(0);
rotations = armEncoder.getPosition();
arm1.setSmartCurrentLimit(35);
arm1.setIdleMode(IdleMode.kBrake);

extenderEncoder.setPosition(0);
extenderrotations=extenderEncoder.getPosition();
extender.setSmartCurrentLimit(19);
extender.setIdleMode(IdleMode.kBrake);
}

@Override
public void testPeriodic (){

      // read PID coefficients from SmartDashboard
      //double p = SmartDashboard.getNumber("P Gain", 0);
      //double i = SmartDashboard.getNumber("I Gain", 0);
      //double d = SmartDashboard.getNumber("D Gain", 0);
      //double iz = SmartDashboard.getNumber("I Zone", 0);
      //double ff = SmartDashboard.getNumber("Feed Forward", 0);
      //double max = SmartDashboard.getNumber("Max Output", 0);
      //double min = SmartDashboard.getNumber("Min Output", 0);
      //double rotations = SmartDashboard.getNumber("Set Rotations", 0);
      
      // if PID coefficients on SmartDashboard have changed, write new values to controller
      /* 
      if((p != kP)) { m_pidController.setP(p); kP = p; }
      if((i != kI)) { m_pidController.setI(i); kI = i; }
      if((d != kD)) { m_pidController.setD(d); kD = d; }
      if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
      if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
      if((max != kMaxOutput) || (min != kMinOutput)) { 
        m_pidController.setOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
      }
      */
  
      /*
       * This code raises the arm by subtracting from the arm position
       * or lowers the arm by adding to the arm position
       * P,I,D values should not be changed while the program is running
       * 
       */
      /*
      if(driver1.getAButton()){
rotations+=0.01;
      }else if(driver1.getBButton()){
rotations-=0.01;
      }  
      */
      
      
      
/*
 * This code controls the arm with the left stick up/down
 */

if(Math.abs(driver2.getLeftY())>0.2){
  rotations=rotations+driver2.getLeftY()*0.04;
} 

/*
 * set limits on range of arm motion
 */
if(rotations<0){
  rotations=0;
}
if(rotations>5){
  rotations=5;
}
/*
 * This code controls the extender with the left stick up/down
 */

 if(Math.abs(driver2.getRightY())>0.2){
  extenderrotations=extenderrotations+driver2.getRightY()*0.16;
} 

/*
 * set limits on range of extender motion
 */
if(extenderrotations<0){
  extenderrotations=0;
}
if(extenderrotations>75){
  extenderrotations=75;
  //MUST MATCH THE ABOVE NUMBER
}

      /**
       * PIDController objects are commanded to a set point using the 
       * SetReference() method.
       * 
       * The first parameter is the value of the set point, whose units vary
       * depending on the control type set in the second parameter.
       * 
       * The second parameter is the control type can be set to one of four 
       * parameters:
       *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
       *  com.revrobotics.CANSparkMax.ControlType.kPosition
       *  com.revrobotics.CANSparkMax.ControlType.kVelocity
       *  com.revrobotics.CANSparkMax.ControlType.kVoltage
       */
      m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
      
      SmartDashboard.putNumber("SetPoint", rotations);
      SmartDashboard.putNumber("Arm Position", armEncoder.getPosition());

      SmartDashboard.putNumber("extSetPoint", extenderrotations);
      SmartDashboard.putNumber("Extender Position", extenderEncoder.getPosition());
      m_pidExtender.setReference(extenderrotations, CANSparkMax.ControlType.kPosition);
      

}








}