/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//network table -> Check if the XY coords are cenetered. If not, turn until they are centered.
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.PistonTimer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Variable that stores half way value of the screen
  int middlePixel = 320;

  // instantiating compressor
  Compressor c = new Compressor(0);

  // compressor status
  boolean enabled = c.enabled();
  boolean pressureSwitch = c.getPressureSwitchValue();
  double current = c.getCompressorCurrent();

  // instantiating solenoid
  // params are the two port numbers for the forward channel and reverse channel
  // respectively
  DoubleSolenoid ballSolenoid = new DoubleSolenoid(0, 1);
  DoubleSolenoid hatchSolenoid = new DoubleSolenoid(2, 3);

  PistonTimer ballPiston = new PistonTimer(ballSolenoid, c, false);
  PistonTimer hatchPiston = new PistonTimer(hatchSolenoid, c, false);

  Boolean aButton, bButton, xButton, yButton;

  Joystick driver;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // when enabled, the PCM will turn on the compressor when the pressure switch is
    // closed
    c.setClosedLoopControl(true);// or false

    // set the state of the valve
    ballSolenoid.set(DoubleSolenoid.Value.kOff);
    hatchSolenoid.set(DoubleSolenoid.Value.kOff);

    driver = new Joystick(0);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // print compressor status to the console
    System.out.println(enabled + "/n" + pressureSwitch + "/n" + current);

    if (ballPiston.movePiston) {
      ballPiston.movePistonFunction();
    } 
    if (hatchPiston.movePiston) {
      hatchPiston.movePistonFunction();
    }
    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
    case kCustomAuto:
      // Put custom auto code here
      break;
    case kDefaultAuto:
    default:
      // Put default auto code here
      break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    // Pneumatic controlls
    aButton = driver.getRawButton(1);
    bButton = driver.getRawButton(2);
    xButton = driver.getRawButton(3);
    yButton = driver.getRawButton(4);

    if (aButton) {
      ballPiston.movePiston = true;
    }
    if (yButton) {
      hatchPiston.movePiston = true;
    }
    /*if (aButton == true) {
      c.setClosedLoopControl(false);
      ballSolenoid.set(DoubleSolenoid.Value.kForward);
    } else if (bButton == true){
      ballSolenoid.set(DoubleSolenoid.Value.kReverse);
      c.setClosedLoopControl(true);
    }
    if (yButton == true) {
      c.setClosedLoopControl(false);
      hatchSolenoid.set(DoubleSolenoid.Value.kForward);
    } else if (xButton == true) {
      hatchSolenoid.set(DoubleSolenoid.Value.kReverse);
      c.setClosedLoopControl(true);
    }/*
    if (bButton == true) {
      c.setClosedLoopControl(true);
    } else if (xButton == true) {
      c.setClosedLoopControl(false);
    }*/
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}