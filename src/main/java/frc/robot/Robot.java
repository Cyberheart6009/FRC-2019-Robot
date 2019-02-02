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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

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

  NetworkTableEntry xEntry;
  NetworkTableEntry yEntry;

  // Variable that stores half way value of the screen
  int middlePixel = 320;

  Boolean aButton, bButton, xButton, yButton;

  Joystick driver;

  NetworkTableInstance inst;
  NetworkTable table;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // Get default instance of automatically created Network Tables
    inst = NetworkTableInstance.getDefault();

    // Get the table within the instance that contains the data
    table = inst.getTable("visionTable");

    // Get X and Y entries
    xEntry = table.getEntry("xEntry");
    yEntry = table.getEntry("yEntry");

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
    //System.out.println(enabled + "/n" + pressureSwitch + "/n" + current);

    //System.out.println("Im In");
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

    if (yButton == true) {
      xEntry.setDouble(xEntry.getDouble(1)+1);
    }
    
    int threshold = 15;

    // drive according to vision input
    if (xEntry.getDouble(0.0) < middlePixel + threshold) {
      System.out.println("Turning Left " + xEntry.getDouble(middlePixel));
      // turn left
    } else if (xEntry.getDouble(0.0) > middlePixel - threshold) {
      System.out.println("Turning Right " + xEntry.getDouble(middlePixel));
      // turn right
    } else {
      System.out.println("Driving Straight " + xEntry.getDouble(middlePixel));
      // drive straight
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
