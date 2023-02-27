// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.









package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.TimedRobot;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;




import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;




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


 private int armtarget;


CANSparkMax left1;
CANSparkMax left2;
CANSparkMax right1;
CANSparkMax right2;
 CANSparkMax arm1;
 CANSparkMax arm2 ;


 private ADIS16470_IMU imu;


UsbCamera camera1;
UsbCamera camera2;
VideoSink server;












RelativeEncoder left1_encoder;
RelativeEncoder right1_encoder;
RelativeEncoder arm_Encoder;


Timer m_timer = new Timer();
private static final String moveauto = "moveauto";
private static final String parkramp = "parkramp";
private String m_autoSelected;
private final SendableChooser<String> m_chooser = new SendableChooser<>();




double Kspeed;
  @Override
  public void robotInit() {
   left1=new CANSparkMax(1, MotorType.kBrushless);
   left2=new CANSparkMax(2, MotorType.kBrushless);
   right1=new CANSparkMax(3, MotorType.kBrushless);
   right2=new CANSparkMax(4, MotorType.kBrushless);
   arm1=new CANSparkMax(5, MotorType.kBrushless);
   arm2=new CANSparkMax(6, MotorType.kBrushless);
   
    imu = new ADIS16470_IMU();
   
 left1_encoder = left1.getEncoder();
 right1_encoder = right1.getEncoder();
 arm_Encoder = arm1.getEncoder();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //m_rightMotor.setInverted(true);
    right1.setInverted(true);
    right2.setInverted(true);
right2.follow(right1);
left2.follow(left1);
arm2.setInverted(true);
arm2.follow(arm1);


armtarget=0;


imu.calibrate();
m_chooser.setDefaultOption("parkramp", parkramp);
m_chooser.addOption("moveauto", moveauto);
SmartDashboard.putData("Auto choices", m_chooser);
SmartDashboard.putNumber("arm_encoder", arm_Encoder.getPosition());




//m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
   // m_leftStick = new Joystick(0);
   //m_rightStick = new Joystick(1);
   driver1=new XboxController(0);
  }//end of robotinit


@Override
public void teleopInit(){
  SmartDashboard.putNumber("Deadzone", 0.2);
  SmartDashboard.putNumber("Xskrenth", 0.5);
  SmartDashboard.putNumber("Yskrenth", 0.5);
  arm_Encoder.setPosition(0);
  arm1.setIdleMode(IdleMode.kBrake);
arm2.setIdleMode(IdleMode.kBrake);
armtarget=0;

exampleSolenoidPH = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8,9);




}// end of teleopInit






  @Override
  public void teleopPeriodic() {
   
    
   
    // m_myRobot.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());
   double Yskrenth = SmartDashboard.getNumber("Yskrenth", 0.5);
   double Xskrenth = SmartDashboard.getNumber("Xskrenth", 0.5);
   double x = Xskrenth* driver1.getRightX();
   double y = -Yskrenth* driver1.getLeftY();
   
   double deadZone = SmartDashboard.getNumber("Deadzone", 0.2);
 if (x < deadZone && x > -deadZone ) {
  x = 0;
 }
 if (y < deadZone && y > -deadZone ) {
  y = 0;
 }


SmartDashboard.putNumber("POV", driver1.getPOV());
if (driver1.getPOV()== 270){


x = -0.05;
}
else if (driver1.getPOV()== 90){
x = 0.05;
}


left1.set(y + x);
right1.set(y - x);


//arm pid control

double error=armtarget-arm_Encoder.getPosition();
double p= SmartDashboard.getNumber("P", .01);
arm1.set(error*p);
SmartDashboard.putNumber("error", error);



if (driver1.getLeftBumper()) {


left1.setIdleMode(IdleMode.kBrake);
left2.setIdleMode(IdleMode.kBrake);
right1.setIdleMode(IdleMode.kBrake);
right2.setIdleMode(IdleMode.kBrake);
}
else {


left1.setIdleMode(IdleMode.kCoast);
left2.setIdleMode(IdleMode.kCoast);
right1.setIdleMode(IdleMode.kCoast);
right2.setIdleMode(IdleMode.kCoast);


}
SmartDashboard.putNumber("Left Drive Encoder", left1_encoder.getPosition());
SmartDashboard.putNumber("Right Drive Encoder", right1_encoder.getPosition());
SmartDashboard.putNumber("Timer:",m_timer.get());
SmartDashboard.putNumber("angle", imu.getAngle());
SmartDashboard.putNumber("X", imu.getAccelX());
SmartDashboard.putNumber("Y", imu.getAccelY());
SmartDashboard.putNumber("Z", imu.getAccelZ());
SmartDashboard.putNumber("arm_encoder", arm_Encoder.getPosition());
}// end of teleopPeriodic


@Override
public void teleopExit(){
SmartDashboard.clearPersistent("Deadzone");

if (driver1.getAButton()) {
  exampleSolenoidPH.set(Value.kForward);
} else {
exampleSolenoidPH.set(Value.kReverse);
}

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
  break;//end of move auto
  case parkramp:


double slow_speed = 0.05;
right1.set(slow_speed);
left1.set(slow_speed);
System.out.println ("here");
//distance to top of hill is ~ 38
if (left1_encoder.getPosition()<38) {
//do nothing
SmartDashboard.putNumber("Y", imu.getAccelY());
SmartDashboard.putNumber("Left Drive Encoder", left1_encoder.getPosition());
SmartDashboard.putNumber("Right Drive Encoder", right1_encoder.getPosition());
}
else {
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






//two new motor controllers






  break;//end of parkramp
}//end of switch statment for auto selection
}//end of autonomous periodic


DoubleSolenoid exampleSolenoidPH;




@Override
public void testInit() {
    // TODO Auto-generated method stub
    super.testInit();
    exampleSolenoidPH = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8,9);
}


@Override
public void testPeriodic() {
    // TODO Auto-generated method stub
    super.testPeriodic();
    if (driver1.getAButton()) {
        exampleSolenoidPH.set(Value.kForward);
    } else {
 exampleSolenoidPH.set(Value.kReverse);
    }
 
}
@Override
public void testExit() {
    // TODO Auto-generated method stub
    super.testExit();
    exampleSolenoidPH.close();
}
}//end of timed robot class


