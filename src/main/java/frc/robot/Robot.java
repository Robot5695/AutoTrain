// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  //private DifferentialDrive m_myRobot;
 // private Joystick m_leftStick;
 // private Joystick m_rightStick;

 // private final MotorController m_leftMotor = new PWMSparkMax(0);
 // private final MotorController m_rightMotor = new PWMSparkMax(1);
XboxController driver1;
CANSparkMax left1 =new CANSparkMax(1, MotorType.kBrushless);
CANSparkMax left2 =new CANSparkMax(2, MotorType.kBrushless);

CANSparkMax right1 =new CANSparkMax(3, MotorType.kBrushless);
CANSparkMax right2 =new CANSparkMax(4, MotorType.kBrushless); 


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
    //m_leftStick = new Joystick(0);
    //m_rightStick = new Joystick(1);
    driver1 =new XboxController(0);
  }

  @Override
  public void teleopPeriodic() {
   // m_myRobot.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());


   double x = driver1.getRightX();
   double y = -driver1.getLeftY();
double range=0.2;

   if (x <range || x>-range){
x =0;
   } else {
//x = (0.2-driver1.getRightX())/0.8
   }

   if (y <range || y>-range){
    y =0;
       } else {
    //x = (0.2-driver1.getRightX())/0.8
       }
   left1.set(y+x);
   right1.set(y-x);
  }
}
