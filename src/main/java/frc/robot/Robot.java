/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//network table -> Check if the XY coords are cenetered. If not, turn until they are centered.
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Servo;

import com.kauailabs.navx.frc.*;

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

  AnalogInput ultrasonic;
  // used in robotSpeed function
  double robotSpeed;
  double oldEncoderCounts = 0.0;
  long oldTime = 0;

  // Variable that stores half way value of the screen
  int middlePixel = 320;

  // SpeedController Object creations - Define all names of motors here
  SpeedController leftFront, leftBack, rightFront, rightBack, intake, elevatorOne, elevatorTwo, lemmeDie;
  Servo servoOne, servoTwo;
  // Speed controller group used for new differential drive class
  SpeedControllerGroup leftChassis, rightChassis, elevator;
  // DifferentialDrive replaces the RobotDrive Class from previous years
  DifferentialDrive chassis;

  // Both encoder sides
  Encoder leftEncoder, rightEncoder, elevatorEncoder;
  // Number of counts per inch, fix elevator value
  // final static double ENCODER_COUNTS_PER_INCH = 13.49;
  final static double ENCODER_COUNTS_PER_INCH = (((1.0/200.0)*(2672.25+2576+2683.5))/3.0);
  //final static double ELEVATOR_ENCODER_COUNTS_PER_INCH = 182.13;
  final static double ELEVATOR_ENCODER_COUNTS_PER_INCH = (((5001.0/70.5)+(4888.0/69.75)+(4968.0/69.75))/3.0);

  Boolean moveIntake = true;
  // Gyroscope Global
  AHRS gyro;

  // instantiating compressor
  Compressor c;
  // instantiating solenoid
  // params are the two port numbers for the forward channel and reverse channel
  // respectively
  DoubleSolenoid ballSolenoid;
  DoubleSolenoid hatchSolenoid;
  // Custom class to enable timed piston extrude and intrude
  boolean doFire;
  double startPistonTime;
  boolean startStartPistonTime = true;
  enum RobotMode {
    HATCH, CARGO
  }
  RobotMode currentRobotMode = RobotMode.HATCH;

  // Creates Joystick buttons
  Boolean aButton, bButton, xButton, yButton, lBumper, rBumper, select, start, leftThumbPush, rightThumbPush;
  Boolean aButtonOp, bButtonOp, xButtonOp, yButtonOp, lBumperOp, rBumperOp, selectOp, startOp, leftThumbPushOp, rightThumbPushOp;
  // Creates the driver's joystick
  Joystick driver, operator;
  double buttonTime = 0;
  boolean startButtonTime = true;
  boolean pressed = false;
  boolean specialPressed = false;
  boolean operatorOverride;

  // Elevator Movement
  double startElevatorTime = 0;

  // Creates the network tables object
  NetworkTableInstance inst;
  // A specific table in network tables
  NetworkTable table;
  // A specific entry for the table
  NetworkTableEntry xEntry;
  NetworkTableEntry yEntry;

  enum AutoMovement {
    STRAIGHT, TURN, VISION, EJECTBALL, EJECTHATCH, ELEVATOR, INTAKE, MODE
  }

  Object[][] autoTemplate = {
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 200, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 90, 0.5 } };

  // Possible Automodes

  /*
   =================================== 
   AUTOMODES FOR SIDE CARGOSHIP HOLDS
   ===================================
   */

  // "1" indicates that the target cargo hold is the closest hold to the alliance
  // station

  Object[][] sideShip1HatchLeft = {
      { AutoMovement.MODE, RobotMode.HATCH },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 214, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 90, 0.5 },
      // Activate Vision
      { AutoMovement.VISION },
      // Elevator
      { AutoMovement.ELEVATOR, ElevatorHeight.HATCH_ONE},
      // Hatch
      { AutoMovement.EJECTHATCH },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, -68, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -90, 0.5 },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 167, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -7.7, 0.5 },
      // TODO: Figure out how to pick up a new ball
      // Maybe use ultrasonic distance- sensors
  };
  Object[][] sideShip2HatchLeft = {
      { AutoMovement.MODE, RobotMode.HATCH },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 236, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 90, 0.5 },
      // Activate Vision
      { AutoMovement.VISION },
      // Hatch
      { AutoMovement.ELEVATOR, ElevatorHeight.HATCH_ONE},
      { AutoMovement.EJECTHATCH },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 68, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -90, 0.5 },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 188.5, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -7.7, 0.5 },
      // TODO: Figure out how to pick up a new ball
      // Maybe use ultrasonic distance- sensors
  };

  Object[][] sideShip3HatchLeft = {
      { AutoMovement.MODE, RobotMode.HATCH },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 258, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 90, 0.5 },
      // Activate Vision
      { AutoMovement.VISION },
      // Hatch
      { AutoMovement.ELEVATOR, ElevatorHeight.HATCH_ONE},
      { AutoMovement.EJECTHATCH },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 68, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -90, 0.5 },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 210, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -7.7, 0.5 },
      // TODO: Figure out how to pick up a new ball
      // Maybe use ultrasonic distance- sensors
  };

  Object[][] sideShip1HatchRight = {
      { AutoMovement.MODE, RobotMode.HATCH },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 214, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -90, 0.5 },
      // Activate Vision
      { AutoMovement.VISION },
      // Hatch
      { AutoMovement.ELEVATOR, ElevatorHeight.HATCH_ONE},
      { AutoMovement.EJECTHATCH },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 68, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 90, 0.5 },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 167, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 7.7, 0.5 },
      // TODO: Figure out how to pick up a new ball
      // Maybe use ultrasonic distance- sensors
  };
  Object[][] sideShip2HatchRight = {
      { AutoMovement.MODE, RobotMode.HATCH },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 236, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -90, 0.5 },
      // Activate Vision
      { AutoMovement.VISION },
      // Hatch
      { AutoMovement.ELEVATOR, ElevatorHeight.HATCH_ONE},
      { AutoMovement.EJECTHATCH },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 68, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 90, 0.5 },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 188.5, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 7.7, 0.5 },
      // TODO: Figure out how to pick up a new ball
      // Maybe use ultrasonic distance- sensors
  };

  Object[][] sideShip3HatchRight = {
      { AutoMovement.MODE, RobotMode.HATCH },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 258, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -90, 0.5 },
      // Activate Vision
      { AutoMovement.VISION },
      // Hatch
      { AutoMovement.ELEVATOR, ElevatorHeight.HATCH_ONE},
      { AutoMovement.EJECTHATCH },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 68, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 90, 0.5 },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 210, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 7.7, 0.5 },
      // TODO: Figure out how to pick up a new ball
      // Maybe use ultrasonic distance- sensors
  };
/* DEPRECATED AUTOMODES (AUTOMODES WITH A BALL AS THE STARTING GAME PIECE)
  Object[][] sideShip1BallLeft = {
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 214, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 90, 0.5 },
      // Activate Vision
      { AutoMovement.VISION },
      // Ball
      { AutoMovement.ELEVATOR, ElevatorHeight.BALL_ONE},
      { AutoMovement.EJECTBALL },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 68, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -90, 0.5 },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 167, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -7.7, 0.5 },
      // TODO: Figure out how to pick up a new ball
      // Maybe use ultrasonic distance- sensors
  };
  Object[][] sideShip2BallLeft = {
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 236, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 90, 0.5 },
      // Activate Vision
      { AutoMovement.VISION },
      // Ball
      { AutoMovement.ELEVATOR, ElevatorHeight.BALL_ONE},
      { AutoMovement.EJECTBALL },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 68, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -90, 0.5 },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 188.5, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -7.7, 0.5 },
      // TODO: Figure out how to pick up a new ball
      // Maybe use ultrasonic distance- sensors
  };

  Object[][] sideShip3BallLeft = {
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 258, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 90, 0.5 },
      // Activate Vision
      { AutoMovement.VISION },
      // Ball
      { AutoMovement.ELEVATOR, ElevatorHeight.BALL_ONE},
      { AutoMovement.EJECTBALL },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 68, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -90, 0.5 },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 210, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -7.7, 0.5 },
      // TODO: Figure out how to pick up a new ball
      // Maybe use ultrasonic distance- sensors
  };

  Object[][] sideShip1BallRight = {
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 214, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -90, 0.5 },
      // Activate Vision
      { AutoMovement.VISION },
      // Ball
      { AutoMovement.ELEVATOR, ElevatorHeight.BALL_ONE},
      { AutoMovement.EJECTBALL },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 68, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 90, 0.5 },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 167, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 7.7, 0.5 },
      // TODO: Figure out how to pick up a new ball
      // Maybe use ultrasonic distance- sensors
  };
  Object[][] sideShip2BallRight = {
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 236, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -90, 0.5 },
      // Activate Vision
      { AutoMovement.VISION },
      // Ball
      { AutoMovement.ELEVATOR, ElevatorHeight.BALL_ONE},
      { AutoMovement.EJECTBALL },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 68, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 90, 0.5 },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 188.5, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 7.7, 0.5 },
      // TODO: Figure out how to pick up a new ball
      // Maybe use ultrasonic distance- sensors
  };

  Object[][] sideShip3BallRight = {
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 258, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -90, 0.5 },
      // Activate Vision
      { AutoMovement.VISION },
      // Ball
      { AutoMovement.ELEVATOR, ElevatorHeight.BALL_ONE},
      { AutoMovement.EJECTBALL },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 68, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 90, 0.5 },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 210, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 7.7, 0.5 },
      // TODO: Figure out how to pick up a new ball
      // Maybe use ultrasonic distance- sensors
  };
  */

  /*
   * =================================== 
   * AUTOMODES FOR FRONT CARGOSHIP HOLDS
   * ===================================
   */
  /* DEPRECATED AUTOMODES (AUTOMODES DROPPING GAMEPIECE ON FRONT HOLDS IN CARGO SHIP)
  Object[][] frontShipBall = {
      // Movement type, Distance, Speed
      // Not driving full distance because vision takes over
      { AutoMovement.STRAIGHT, 168, 1 },
      // Activate Vision
      { AutoMovement.VISION },
      // Ball
      { AutoMovement.ELEVATOR, ElevatorHeight.BALL_ONE},
      { AutoMovement.EJECTBALL },
      // Movement type, Distance, Speed
  };

  Object[][] frontShipHatch = {
    // Movement type, Distance, Speed
    // Not driving full distance because vision takes over
    { AutoMovement.STRAIGHT, 168, 1 },
    // Activate Vision
    { AutoMovement.VISION },
    // Ball
    { AutoMovement.ELEVATOR, ElevatorHeight.BALL_ONE},
    { AutoMovement.EJECTHATCH },
    // Movement type, Distance, Speed
};
*/

/* DEPCRATED AUTOMODES (AUTOMODES WITH A BALL AS THE STARTING GAME PIECE)
//Automode 5 (Left Rocket Cargo)
Object[][] autoRockeBallLeft = {
  {AutoMovement.STRAIGHT, 181, 1},
  {AutoMovement.TURN, 90, -0.5},
  {AutoMovement.STRAIGHT, 50, 1},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, "BALL_ONE"},
  { AutoMovement.ELEVATOR, ElevatorHeight.BALL_ONE},
  {AutoMovement.EJECTBALL},
  {AutoMovement.STRAIGHT, 10, -1},
  {AutoMovement.TURN, 90, 0.5},
  {AutoMovement.STRAIGHT, 130, -1},
  {AutoMovement.TURN, 39.5, -0.5},
  {AutoMovement.STRAIGHT, 110, -1}
};

//Automode 6 (Right Rocket Cargo)
Object[][] autoRocketBallRight = {
  {AutoMovement.STRAIGHT, 181, 1},
  {AutoMovement.TURN, 90, 0.5},
  {AutoMovement.STRAIGHT, 50, 1},
  {AutoMovement.VISION},
  { AutoMovement.ELEVATOR, ElevatorHeight.BALL_ONE},
  {AutoMovement.EJECTBALL},
  {AutoMovement.STRAIGHT, 10, -1},
  {AutoMovement.TURN, 90, -0.5},
  {AutoMovement.STRAIGHT, 130, -1},
  {AutoMovement.TURN, 39.5, 0.5},
  {AutoMovement.STRAIGHT, 110, -1}
};

//Automode 7 (Left Lower Rocket Hatch) Starting position is on edge before ramp
Object[][] autoRocketHatchLeftLower = {
  {AutoMovement.STRAIGHT, 34, 1},
  {AutoMovement.TURN, 17.1, 0.5},
*/
//Automode 7 (Left Lower Rocket Hatch) Starting position is on line seperating higher levels
Object[][] autoRocketHatchLeftLower = {
  { AutoMovement.MODE, RobotMode.HATCH },
  {AutoMovement.STRAIGHT, 34, 1},
  {AutoMovement.TURN, 17.1, 0.5},
  {AutoMovement.STRAIGHT, 150, 1},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_ONE},
  {AutoMovement.EJECTHATCH},

  {AutoMovement.STRAIGHT, 12.8, -1},
  {AutoMovement.TURN, 140, -0.5},
  {AutoMovement.STRAIGHT, 206, 1},
  {AutoMovement.STRAIGHT, 10, -1},
  {AutoMovement.TURN, 20.5, 0.5},
  {AutoMovement.STRAIGHT, 94, -1},
  {AutoMovement.TURN, 20.5, -0.5},
  {AutoMovement.STRAIGHT, 211, -1},
  {AutoMovement.TURN, 32, 0.5},
  {AutoMovement.STRAIGHT, 60, 1},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_ONE},
  {AutoMovement.EJECTHATCH}
};

//Automode 8 (Left Middle Rocket Hatch) Starting position is on edge before ramp
Object[][] autoRocketHatchLeftMiddle = {
  {AutoMovement.MODE, RobotMode.HATCH },
  {AutoMovement.STRAIGHT, 34, 1},
  {AutoMovement.TURN, 17.1, 0.5},
  {AutoMovement.STRAIGHT, 150, 1},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_TWO},
  {AutoMovement.EJECTHATCH},

  {AutoMovement.STRAIGHT, 12.8, -1},
  {AutoMovement.TURN, 140, -0.5},
  {AutoMovement.STRAIGHT, 206, 1},
  {AutoMovement.STRAIGHT, 10, -1},
  {AutoMovement.TURN, 20.5, 0.5},
  {AutoMovement.STRAIGHT, 94, -1},
  {AutoMovement.TURN, 20.5, -0.5},
  {AutoMovement.STRAIGHT, 211, -1},
  {AutoMovement.TURN, 32, 0.5},
  {AutoMovement.STRAIGHT, 60, 1},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_TWO},
  {AutoMovement.EJECTHATCH}
};

//Automode 9 (Left Upper Rocket Hatch) Starting position is on edge before ramp
Object[][] autoRocketHatchLeftUpper = {
  {AutoMovement.MODE, RobotMode.HATCH },
  {AutoMovement.STRAIGHT, 34, 1},
  {AutoMovement.TURN, 17.1, 0.5},
  {AutoMovement.STRAIGHT, 150, 1},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_THREE},
  {AutoMovement.EJECTHATCH},

  {AutoMovement.STRAIGHT, 12.8, -1},
  {AutoMovement.TURN, 140, -0.5},
  {AutoMovement.STRAIGHT, 206, 1},
  {AutoMovement.STRAIGHT, 10, -1},
  {AutoMovement.TURN, 20.5, 0.5},
  {AutoMovement.STRAIGHT, 94, -1},
  {AutoMovement.TURN, 20.5, -0.5},
  {AutoMovement.STRAIGHT, 211, -1},
  {AutoMovement.TURN, 32, 0.5},
  {AutoMovement.STRAIGHT, 60, 1},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_THREE},
  {AutoMovement.EJECTHATCH}
};

//Automode 10 (Right Lower Rocket Hatch) Starting position is on edge before ramp
Object[][] autoRocketHatchRightLower = {
  { AutoMovement.MODE, RobotMode.HATCH },
  {AutoMovement.STRAIGHT, 34, 1},
  {AutoMovement.TURN, 17.1, -0.5},
  {AutoMovement.STRAIGHT, 150, 1},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_ONE},
  {AutoMovement.EJECTHATCH},

  {AutoMovement.STRAIGHT, 12.8, -1},
  {AutoMovement.TURN, 140, 1},
  {AutoMovement.STRAIGHT, 206, 1},
  {AutoMovement.STRAIGHT, 10, -1},
  {AutoMovement.TURN, 20.5, -0.5}, 
  {AutoMovement.STRAIGHT, 94, -1},
  {AutoMovement.TURN, 20.5, 0.5},
  {AutoMovement.STRAIGHT, 211, -1},
  {AutoMovement.TURN, 32, -0.5},
  {AutoMovement.STRAIGHT, 60, 1}, 
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_ONE},
  {AutoMovement.EJECTHATCH}
};

//Automode 11 (Right Middle Rocket Hatch) Starting position is on edge before ramp
Object[][] autoRocketHatchRightMiddle = {
  { AutoMovement.MODE, RobotMode.HATCH },
  {AutoMovement.STRAIGHT, 34, 1},
  {AutoMovement.TURN, 17.1, -0.5},
  {AutoMovement.STRAIGHT, 150, 1},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_TWO},
  {AutoMovement.EJECTHATCH},

  {AutoMovement.STRAIGHT, 12.8, -1},
  {AutoMovement.TURN, 140, 1},
  {AutoMovement.STRAIGHT, 206, 1},
  {AutoMovement.STRAIGHT, 10, -1},
  {AutoMovement.TURN, 20.5, -0.5}, 
  {AutoMovement.STRAIGHT, 94, -1},
  {AutoMovement.TURN, 20.5, 0.5},
  {AutoMovement.STRAIGHT, 211, -1},
  {AutoMovement.TURN, 32, -0.5},
  {AutoMovement.STRAIGHT, 60, 1}, 
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_TWO},
  {AutoMovement.EJECTHATCH}
};

//Automode 12 (Right Upper Rocket Hatch) Starting position is on edge before ramp
Object[][] autoRocketHatchRightUpper = {
  {AutoMovement.MODE, RobotMode.HATCH },
  {AutoMovement.STRAIGHT, 34, 1},
  {AutoMovement.TURN, 17.1, -0.5},
  {AutoMovement.STRAIGHT, 150, 1},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_THREE},
  {AutoMovement.EJECTHATCH},

  {AutoMovement.STRAIGHT, 12.8, -1},
  {AutoMovement.TURN, 140, 1},
  {AutoMovement.STRAIGHT, 206, 1},
  {AutoMovement.STRAIGHT, 10, -1},
  {AutoMovement.TURN, 20.5, -0.5}, 
  {AutoMovement.STRAIGHT, 94, -1},
  {AutoMovement.TURN, 20.5, 0.5},
  {AutoMovement.STRAIGHT, 211, -1},
  {AutoMovement.TURN, 32, -0.5},
  {AutoMovement.STRAIGHT, 60, 1}, 
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_THREE},
  {AutoMovement.EJECTHATCH}
};
  // Dictates the current auto that is selected
  Object[][] selectedAuto;
  // Indicates what step of auto the robot is on
  int autoStep;
  // Indicates when auto should stop
  boolean autoStop;

  enum ElevatorHeight {
    HATCH_ONE, HATCH_TWO, HATCH_THREE, BALL_ONE, BALL_TWO, BALL_THREE
  }
  double elevatorHeight;

  int invertControls = 1;

  // TODO: Modify these temp variables
  Boolean elevatorLimitSwitch = false;

  boolean setStartAngle = true;
  double startAngle;



  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // Start ultrasonic sensor
    // ultrasonic = new AnalogInput(0);
    // Defines all the ports of each of the motors
    leftFront = new Spark(0);
    leftBack = new Spark(1);
    rightFront = new Spark(2);
    rightBack = new Spark(3);
    intake = new Spark(4);
    elevatorOne = new Spark(5);
    elevatorTwo = new Spark(6);
    lemmeDie = new Spark(7);
    servoOne = new Servo(8);
    servoTwo = new Servo(9);

    // Defines the left and right SpeedControllerGroups for our DifferentialDrive
    // class
    leftChassis = new SpeedControllerGroup(leftFront, leftBack);
    rightChassis = new SpeedControllerGroup(rightFront, rightBack);
    elevator = new SpeedControllerGroup(elevatorOne, elevatorTwo);
    // Inverts the right side of the drive train to account for the motors being
    // physically flipped
    leftChassis.setInverted(true);
    // Defines our DifferentalDrive object with both sides of our drivetrain
    chassis = new DifferentialDrive(leftChassis, rightChassis);

    // Setting encoder ports
    leftEncoder = new Encoder(0, 1);
    rightEncoder = new Encoder(2, 3);
    elevatorEncoder = new Encoder(4, 5);

    // Initialize gyroscope object
    gyro = new AHRS(SPI.Port.kMXP);

    // Initializes compressor
    c = new Compressor(0);
    // Compressor solenoids
    hatchSolenoid = new DoubleSolenoid(1, 0);
    ballSolenoid = new DoubleSolenoid(2, 3);

    // Sets the joystick port
    driver = new Joystick(0);
    operator = new Joystick(1);
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

    // Maps a joystick to a variable
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
    // System.out.println(enabled + "/n" + pressureSwitch + "/n" + current);

    // System.out.println("Im In");
    updateSmartDashboard();

    if (pressed) {
      if (buttonTime + 200 < System.currentTimeMillis()) {
        pressed = false;
      }
    }

    if (specialPressed) {
      if (buttonTime + 200 < System.currentTimeMillis()) {
        pressed = false;
      }
    }

    if (doFire) {
      fire();
    }

    if (startElevatorTime + 200 < System.currentTimeMillis() && startElevatorTime > 1) {
      if (elevatorMovement(ElevatorHeight.HATCH_ONE)) {
        startElevatorTime = 0;
      }
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
     * Movement Type: (AutoMovement) selectedAuto[autoStep][1] Movement Special:
     * (double) selectedAuto[autoStep][2] Movement Speed: (double)
     * selectedAuto[autoStep][3]
     * 
     */

    // Stops the entire robot code when autoStop = true;
    if (!autoStop) {
      switch ((AutoMovement) selectedAuto[autoStep][0]) {
        case MODE:
          currentRobotMode = (RobotMode) selectedAuto[autoStep][1];
          break;
        case STRAIGHT:
          if (getDistance() < ((double) selectedAuto[autoStep][1]) - 10) { // Forwards
            chassis.arcadeDrive((double) selectedAuto[autoStep][2], 0);
          } else if (getDistance() > (double) selectedAuto[autoStep][1] + 10) { // Backwards
            chassis.arcadeDrive((-(double) selectedAuto[autoStep][2]), 0);
          } else { // Destination Reached
            resetEncoders();
            autoStep++;
          }
          break;
        case TURN:
          if (setStartAngle) {
            startAngle = getAngle();
            setStartAngle = false;
          }
          if (getAngle() - startAngle < ((double) selectedAuto[autoStep][1] - 10)) {
            leftBack.set((double) selectedAuto[autoStep][2]);
            leftFront.set((double) selectedAuto[autoStep][2]);
            rightBack.set(-(double) selectedAuto[autoStep][2]);
            rightFront.set(-(double) selectedAuto[autoStep][2]);
          } else if (getAngle() - startAngle > ((double) selectedAuto[autoStep][1] + 10)) {
            leftBack.set(-(double) selectedAuto[autoStep][2]);
            leftFront.set(-(double) selectedAuto[autoStep][2]);
            rightBack.set((double) selectedAuto[autoStep][2]);
            rightFront.set((double) selectedAuto[autoStep][2]);
          } else { // Turn Complete
            setStartAngle = true;
            resetEncoders();
            autoStep++;
          }
          break;
        case ELEVATOR:
          elevatorMovement((ElevatorHeight) selectedAuto[autoStep][1]);
        case EJECT:
          doFire = true;
          break;
        case VISION:
          if (true) {
            cameraControl();
          } else {
            // TODO: Add a coninuation section for auto code
            autoStep++;
          }
          break;
      }
    }

  }

  /**
   * This function is called as teleop is Initiated
   */
  @Override
  public void teleopInit() {
    super.teleopInit();
    operatorOverride = false;
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // Main Robot Movement
    chassis.arcadeDrive(-driver.getX(), (invertControls*driver.getY()));

    // Driver Input Buttons
    aButton = driver.getRawButton(1);
    bButton = driver.getRawButton(2);
    xButton = driver.getRawButton(3);
    yButton = driver.getRawButton(4);
    lBumper = driver.getRawButton(5);
		rBumper = driver.getRawButton(6);
		select = driver.getRawButton(7);
		start = driver.getRawButton(8);
		leftThumbPush = driver.getRawButton(9);
    rightThumbPush = driver.getRawButton(10);

    if (start) {
      if (invertControls == -1) {
        invertControls = 1;
      } else if (invertControls == 1) {
        invertControls = -1;
      } else {
        System.out.println("Something went wrong when inverting the controls");
      }
    }
    if (lBumper){
    } 
    if (rBumper) {
      invertControls = -1;
    } else {
      invertControls = 1;
    }

    // OPERATOR CONTROLS BEGINS
    aButtonOp = operator.getRawButton(1);
    bButtonOp = operator.getRawButton(2);
    xButtonOp = operator.getRawButton(3);
    yButtonOp = operator.getRawButton(4);
    lBumperOp = operator.getRawButton(5);
		rBumperOp = operator.getRawButton(6);
		selectOp = operator.getRawButton(7);
		startOp = operator.getRawButton(8);
		leftThumbPushOp = operator.getRawButton(9);
    rightThumbPushOp = operator.getRawButton(10);

    if (select) {
      if (!specialPressed) {
        operatorOverride = true;
        specialPressed = true;
        buttonTime = System.currentTimeMillis();
      }
    }

    if (operatorOverride) {
      elevator.set(operator.getY());
      if (xButtonOp) {
        doFire = true;
      }
    }

    lemmeDie.set(-operator.getRawAxis(5));

    if (operator.getRawAxis(2) > 0.1 || operator.getRawAxis(3) > 0.1) {
      intake.set(operator.getRawAxis(2));
      intake.set(-operator.getRawAxis(3));
    }

    if (rBumperOp) {
      if (!specialPressed) {
        switchMode();
        specialPressed = true;
        buttonTime = System.currentTimeMillis();
      }
    }

    if (!pressed) {
      if (currentRobotMode == RobotMode.CARGO) {
        if (aButtonOp) {
          elevatorMovement(ElevatorHeight.BALL_ONE);
        }
        if (bButtonOp) {
          elevatorMovement(ElevatorHeight.BALL_TWO);
        }
        if (yButtonOp) {
          elevatorMovement(ElevatorHeight.BALL_THREE);
        }
      } else {
        if (aButtonOp) {
          elevatorMovement(ElevatorHeight.HATCH_ONE);
        }
        if (bButtonOp) {
          elevatorMovement(ElevatorHeight.HATCH_TWO);
        }
        if (yButtonOp) {
          elevatorMovement(ElevatorHeight.HATCH_THREE);
        }
      }
      pressed = true;
      buttonTime = System.currentTimeMillis();
    }

    // The old control scheme
    /**
    if (aButtonOp) {
      doFire = true;
    }
    if (xButtonOp) {
      if (!pressed) {
        switchMode();
        pressed = true;
        buttonTime = System.currentTimeMillis();
      }
    }
    if (bButtonOp){
      lemmeDie.set(1);
    } else if (yButtonOp) {
      lemmeDie.set(-1);
    }
    if (lBumperOp) {
      intake.set(1);
    } else if (rBumperOp) {
      intake.set(-1);
    } else {
      intake.set(0);
    }
    */

    // This block of code is for testing the servos
    /*
    if (startOp) {
      servoOne.setAngle(0);
      servoTwo.setAngle(0);
    }
    if (rightThumbPushOp) {
      servoOne.setAngle(servoOne.getAngle() +1);
      servoTwo.setAngle(servoTwo.getAngle() +1);
    }
    if (leftThumbPushOp) {
      servoOne.setAngle(servoOne.getAngle() -1);
      servoTwo.setAngle(servoTwo.getAngle() -1);
    } */



    /*if (aButtonOp) {
    } if (bButtonOp) {
      intakeLift.set(0);
    }*/

    // Test Code for the pistons
    /*
     if (aButton == true) { 
      c.setClosedLoopControl(false);
      ballSolenoid.set(DoubleSolenoid.Value.kForward); 
    } else if (bButton == true){
      ballSolenoid.set(DoubleSolenoid.Value.kReverse);
      c.setClosedLoopControl(true); 
    } if (yButton == true) {
      c.setClosedLoopControl(false);
      hatchSolenoid.set(DoubleSolenoid.Value.kForward); 
    } else if (xButton == true) { 
      hatchSolenoid.set(DoubleSolenoid.Value.kReverse);
      c.setClosedLoopControl(true); 
    } /*if (bButton == true) {
      c.setClosedLoopControl(true); 
    } else if (xButton == true) {
      c.setClosedLoopControl(false); }
    
      // Test code for the Auto
       /*
     * int threshold = 15; // drive according to vision input if
     * (xEntry.getDouble(0.0) < middlePixel + threshold) {
     * System.out.println("Turning Left " + xEntry.getDouble(middlePixel)); // turn
     * left } else if (xEntry.getDouble(0.0) > middlePixel - threshold) {
     * System.out.println("Turning Right " + xEntry.getDouble(middlePixel)); // turn
     * right } else { System.out.println("Driving Straight " +
     * xEntry.getDouble(middlePixel)); // drive straight }
     */
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  // Converts encoder counts into inches
  public double getDistance() {
    //return ((double) (leftEncoder.get() + rightEncoder.get()) / (ENCODER_COUNTS_PER_INCH * 2));
    return 1;
  }

  // Resets both encoders with one function
  public void resetEncoders() {
    //leftEncoder.reset();
    //rightEncoder.reset();
  }

  // gets the current gyro angle
  public double getAngle() {
    // Add in heading code
    return gyro.getAngle();
  }

  // Takes camera input and converts it into a robot action
  public void cameraControl() {
    // Sets the threshold for vision
    int threshold = 15;
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

    /*
     * // Ultrasonic Distance Auto Foolproof Mech. Compares middle, right, and left
     * // distances and finds the open cargo. if (ultraMid > ultraLef && ultraMid >
     * ultraRig) { // drive straight } else if (ultraMid < ultraLef || ultraMid <
     * ultraRig || (ultraMid < ultraLef && ultraMid < ultraRig)) { if (ultraLef >
     * ultraRig) { // adjust to left } else if (ultraLef < ultraRig) { // adjust to
     * right } else { ; }
     * 
     * }
     */

    /*
     * this.leftDistance = ultraLeft.getRangeInches();
     * System.out.println(this.leftDistance); }
     * 
     * if (ultraMiddle.getRangeInches() < ultraRight.getRangeInches()) {
     * this.rightDistance = ultraRight.getRangeInches();
     * System.out.println(this.rightDistance); }
     * 
     * if (this.leftDistance > this.rightDistance) { // adjust left } else if
     * (this.leftDistance < this.rightDistance) { // adjust right }
     */
  }

  // Get elevator height
  private double getElevatorHeight() {
    return (double) (elevatorEncoder.get() / ELEVATOR_ENCODER_COUNTS_PER_INCH);
    //return 1;
  }

  // Calculates the robotSpeed
  public double robotSpeed() {
    // Calculates current speed of the robot in m/s
    //robotSpeed = ((getDistance() - oldEncoderCounts) / (System.currentTimeMillis() - oldTime)) * 0.0254;
    //oldTime = System.currentTimeMillis();
    //oldEncoderCounts = getDistance();
    //return (double) robotSpeed;
    return 1;
  }

  // Updates SmartDashboard
  public void updateSmartDashboard() {
    SmartDashboard.putData("gyro", gyro);
    SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
    SmartDashboard.putNumber("Gyro Rate", gyro.getRate());

    SmartDashboard.putNumber("Encoder Distance", getDistance());
    SmartDashboard.putNumber("Left Encoder Distance", leftEncoder.getDistance());
    SmartDashboard.putNumber("Right Encoder Distance", rightEncoder.getDistance());
    
    SmartDashboard.putNumber("ServoOne", servoOne.getAngle());
    SmartDashboard.putNumber("ServoTwo", servoTwo.getAngle());

    SmartDashboard.putNumber("Robot Speed", robotSpeed());

    String xEnt = xEntry.toString();
    SmartDashboard.putString("X Entry", xEnt);

    SmartDashboard.putNumber("Elevator Encoders", elevatorEncoder.get());
    SmartDashboard.putNumber("Elevator Height", getElevatorHeight());

    SmartDashboard.putString("Current Robot mode", currentRobotMode.name());

    SmartDashboard.putNumber("Ultrasonic Distance", getUltrasonicDistance());
  }

  public double getUltrasonicDistance() {
    return (double) (((ultrasonic.getAverageVoltage() * 1000) / 238.095) + 9.0);
  }

  public boolean elevatorMovement(ElevatorHeight level) {
    double firstHeight = 0;
    switch (level) {
      case HATCH_ONE:
        elevatorHeight = firstHeight;
        break;
      case HATCH_TWO:
        elevatorHeight = firstHeight + 24;
        break;
      case HATCH_THREE:
        elevatorHeight = firstHeight + 48;
        break;
      case BALL_ONE:
        elevatorHeight = firstHeight + 10.25;
        break;
      case BALL_TWO:
        elevatorHeight = firstHeight + 38.25;
        break;
      case BALL_THREE:
        elevatorHeight = firstHeight + 66.25;
        break;
      default:
        break;
    }
    double x = getElevatorHeight();
    double domain = elevatorHeight;
    elevator.set(((0-1)/((0-(domain/2))*(0-(domain/2))))*((x-(domain/2))*(x-(domain/2))) + 1);

    if (x >= elevatorHeight) {
      return true;
    } else {
      return false;
    }
  }

  public boolean liftFire(ElevatorHeight level) {
    if (elevatorMovement(level)) {
      doFire = true;
      startElevatorTime = System.currentTimeMillis();
      return true;
    } else {
      return false;
    }
  }

  public void servoClose() {
    servoOne.setAngle(getMyAngle(270/2));
    // This number is different to confuse future programmers
    servoTwo.setAngle(getMyAngle(135));
  }
  public void servoOpen() {
    servoOne.setAngle(getMyAngle((270/2)-90));
    servoTwo.setAngle(getMyAngle(135+90));
  }

  public double getMyAngle(double angle) {
    double answer = angle*(0.666666);
    //System.out.println(answer);
    return answer;
  }

  public void switchMode() {
    if (currentRobotMode == RobotMode.HATCH) {
      currentRobotMode = RobotMode.CARGO;
      servoClose();
      System.out.println("We are in Cargo Mode");
    } else {
      currentRobotMode = RobotMode.HATCH;
      servoOpen();
      System.out.println("We are in Hatch Mode");
    }
  }

  public void startPistonTimer() {
    startPistonTime = System.currentTimeMillis();
  }

  public void ultrasonicStop() {
    if (getUltrasonicDistance() <= 1) {
      chassis.stopMotor();
    }
  }

  public void fire() {
    if (startStartPistonTime) {
        startPistonTimer();
        startStartPistonTime = false;
    }
    if (currentRobotMode == RobotMode.HATCH) {
      if (System.currentTimeMillis() < this.startPistonTime + 300) {
        c.setClosedLoopControl(false);
        hatchSolenoid.set(DoubleSolenoid.Value.kForward);
        servoClose();
      } else {
        hatchSolenoid.set(DoubleSolenoid.Value.kReverse);
        c.setClosedLoopControl(true);
        startStartPistonTime = true;
        doFire = false;
        servoOpen();
      }
    } else {
      if (System.currentTimeMillis() < this.startPistonTime + 150) {
        servoOpen();
      } else if (System.currentTimeMillis() < this.startPistonTime + 650) {
        c.setClosedLoopControl(false);
        ballSolenoid.set(DoubleSolenoid.Value.kForward);
      }
      else {
        ballSolenoid.set(DoubleSolenoid.Value.kReverse);
        c.setClosedLoopControl(true);
        startStartPistonTime = true;
        doFire = false;
        servoClose();
      }
    }

  }
}
