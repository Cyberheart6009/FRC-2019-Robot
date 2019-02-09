/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//network table -> Check if the XY coords are cenetered. If not, turn until they are centered.
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.*;
// Custom Class

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.PistonTimer;

// Our New Imports
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Spark;

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

  // SpeedController Object creations - Define all names of motors here
  SpeedController leftFront, leftBack, rightFront, rightBack, gripper;
  // Speed controller group used for new differential drive class
  SpeedControllerGroup leftChassis, rightChassis;
  // DifferentialDrive replaces the RobotDrive Class from previous years
  DifferentialDrive chassis;
  
  // Both encoder sides
  Encoder leftEncoder, rightEncoder;
  // Number of counts per inch
  final static double ENCODER_COUNTS_PER_INCH = 13.49;

  // Gyroscope Global
  AHRS gyro;
  // instantiating solenoid
  // params are the two port numbers for the forward channel and reverse channel
  // respectively
  DoubleSolenoid ballSolenoid = new DoubleSolenoid(0, 1);
  DoubleSolenoid hatchSolenoid = new DoubleSolenoid(2, 3);

  PistonTimer ballPiston = new PistonTimer(ballSolenoid, c, false);
  PistonTimer hatchPiston = new PistonTimer(hatchSolenoid, c, false);

  // Creates Joystick buttons
  Boolean aButton, bButton, xButton, yButton;
  // Creates the driver's joystick
  Joystick driver;


  // Creates the network tables object
  NetworkTableInstance inst;
  // A specific table in network tables
  NetworkTable table;
  // A specific entry for the table
  NetworkTableEntry xEntry;
  NetworkTableEntry yEntry;

  enum AutoMovement {
    STRAIGHT,
    TURN,
    VISION
  }
  Object[][] autoTemplate = {
    // Movement type, Distance, Speed
    {AutoMovement.STRAIGHT, 200, 1},
    // Movement type, Rotation, Speed
    {AutoMovement.TURN, 90, 0.5}
  };
  // Dictates the current auto that is selected
  Object[][] selectedAuto;
  // Indicates what step of auto the robot is on
  int autoStep;
  // Indicates when auto should stop
  boolean autoStop;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);


    // Defines all the ports of each of the motors
		leftFront = new Spark(0);
		leftBack = new Spark(1);
		rightFront = new Spark(2);
		rightBack = new Spark(3);
		gripper = new Spark(4);
		// Defines the left and right SpeedControllerGroups for our DifferentialDrive class
		leftChassis = new SpeedControllerGroup(leftFront, leftBack);
		rightChassis = new SpeedControllerGroup(rightFront, rightBack);
		// Inverts the right side of the drive train to account for the motors being physically flipped
		rightChassis.setInverted(true);
		// Defines our DifferentalDrive object with both sides of our drivetrain
    chassis = new DifferentialDrive(leftChassis, rightChassis);

    // Setting encoder ports
    leftEncoder = new Encoder(0,1);
		rightEncoder = new Encoder(2,3);
    
    // Initialize gyroscope object
    gyro = new AHRS(SPI.Port.kMXP); 

    // Sets the joystick port
    driver = new Joystick(0);
    // Controls
    aButton = driver.getRawButton(1);
    bButton = driver.getRawButton(2);
    xButton = driver.getRawButton(3);
    yButton = driver.getRawButton(4);


    // Get default instance of automatically created Network Tables
    inst = NetworkTableInstance.getDefault();
    // Get the table within the instance that contains the data
    table = inst.getTable("visionTable");
    // Get X and Y entries
    xEntry = table.getEntry("xEntry");
    yEntry = table.getEntry("yEntry");
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

    // The step that auto is on
    autoStep = 0;

    // Which auto are we using?
    selectedAuto = autoTemplate;

    // Has the auto finished?
    autoStop = false;

    // Reset the encoders before game begins
    resetEncoders();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    /** 
     *  Movement Type: (AutoMovement) selectedAuto[autoStep][1]
     *  Movement Special: (double) selectedAuto[autoStep][2]
     *  Movement Speed: (double) selectedAuto[autoStep][3]
     */

    // Sets the threshold for vision
    int threshold = 15;

     // Stops the entire robot code when autoStop = true;
    if (!autoStop){
      // If the STRAIGHT movement is selected
      if ((AutoMovement) selectedAuto[autoStep][1] == AutoMovement.STRAIGHT) {
        if (getDistance() < ((double) selectedAuto[autoStep][2]) - 10){         // Forwards
          chassis.arcadeDrive((double) selectedAuto[autoStep][3], 0);
        } else if (getDistance() > (double) selectedAuto[autoStep][2] + 10){    // Backwards
          chassis.arcadeDrive((-(double) selectedAuto[autoStep][3]), 0);
        } else {      // Destination Reached
          resetEncoders();
          autoStep++;
        }
      }
      // If the TURN movement is selected
      else if ((AutoMovement) selectedAuto[autoStep][1] == AutoMovement.TURN){
        if (getAngle() < ((double) selectedAuto[autoStep][2] - 10) || getAngle() < ((double) selectedAuto[autoStep][2] + 10)) {   // Turning code
          chassis.arcadeDrive((double) selectedAuto[autoStep][3], (double) selectedAuto[autoStep][2]);
        } else {    // Turn Complete
          resetEncoders();
          autoStep++;
        }
      }
      // If the VISION movement is selected
      else if ((AutoMovement) selectedAuto[autoStep][1] == AutoMovement.VISION){
        if (true) {
          if (xEntry.getDouble(0.0) < middlePixel + threshold) {
            chassis.arcadeDrive(1.0, -10);
            System.out.println("Turning Left " + xEntry.getDouble(middlePixel));
            // turn left
          } else if (xEntry.getDouble(0.0) > middlePixel - threshold) {
            chassis.arcadeDrive(1.0, 10);
            System.out.println("Turning Right " + xEntry.getDouble(middlePixel));
            // turn right
          } else {
            chassis.arcadeDrive(1.0, 0);
            System.out.println("Driving Straight " + xEntry.getDouble(middlePixel));
            // drive straight
          }
        }
        else {
          // TODO: Add a coninuation section for auto code
          autoStep++;
        }
      }
    }
  }
  
  /**
   * This function is called as teleop is Initiated
   */
  @Override
  public void teleopInit() {
    super.teleopInit();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    if (yButton == true) {
      xEntry.setDouble(xEntry.getDouble(1)+1);
    }

    /*
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
    }*/
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
    chassis.arcadeDrive(driver.getX(), driver.getY());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public double getDistance(){
		return ((double)(leftEncoder.get() + rightEncoder.get()) / (ENCODER_COUNTS_PER_INCH * 2));
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }
  
  public double getAngle() {
    // Add in heading code
    return gyro.getAngle();
  }
}