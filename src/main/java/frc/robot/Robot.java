/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//network table -> Check if the XY coords are cenetered. If not, turn until they are centered.
//mjpg stream (trhough TCDSB internet) http://10.16.149.192:8081 or 8080/?action=stream
//mjpg command "mjpg_streamer -i 'input_uvc.so -br 249' -o output_http.so"
//pi connection ssh raspberrypi.local -l pi
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
//import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.cscore.UsbCamera;

import com.kauailabs.navx.frc.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static final String leftShipShort = "Left Ship Short";
  private static final String leftShipMiddle = "Left Ship Middle";
  private static final String leftShipLong = "Left Ship Long";

  private static final String rightShipShort = "Right Ship Short";
  private static final String rightShipMiddle = "Right Ship Middle";
  private static final String rightShipLong = "Right Ship Long";

  private static final String leftRocketLow = "Left Rocket Low";
  private static final String leftRocketMedium = "Left Rocket Medium";
  private static final String leftRocketHigh = "Left Rocket High";

  private static final String rightRocketLow = "Right Rocket Low";
  private static final String rightRocketMedium = "Right Rocket Medium";
  private static final String rightRocketHigh = "Right Rocket High";

  private static final String visionAutoTesting = "visionAutoTest";
  private static final String rightRocketMiddleHigh = "Right Rocket Middle to High";
  private static final String leftRocketMiddleHigh = "Left Rocket Middle to High";

  private static final String driverControlled = "No Auto";
  double oldX = 0.0;

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // AnalogInput ultrasonic;
  // used in robotSpeed function
  double robotSpeed;
  double oldEncoderCounts = 0.0;
  long oldTime = 0;

  // Variable that stores half way value of the screen
  int middlePixel = 320;

  // SpeedController Object creations - Define all names of motors here
  SpeedController leftFront, leftBack, rightFront, rightBack, intake, elevatorOne, elevatorTwo, lemmeDie;
  Servo servoOne, servoTwo, cameraX, cameraY;
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
  DoubleSolenoid ballSolenoid, hatchSolenoid, hatchGrabSolenoid, intakeSolenoid;
  DoubleSolenoid frontClimb, backClimb;
  // Custom class to enable timed piston extrude and intrude
  boolean doFire;
  double startPistonTime;
  boolean startStartPistonTime = true;
  enum RobotMode {
    HATCH, CARGO
  }
  RobotMode currentRobotMode = RobotMode.HATCH;

  // Creates Joystick buttons
  Boolean aButton, aButtonPressed, bButton, bButtonPressed, xButton, yButton, lBumper, rBumper, select, start, leftThumbPush, rightThumbPush;
  Boolean aButtonOp, bButtonOp, xButtonOp, yButtonOp, lBumperOp, rBumperOp, selectOp, startOp, leftThumbPushOp, rightThumbPushOp, rBumperOpPressed;
  // Creates the driver's joystick
  Joystick driver, operator;
  boolean operatorOverride;

  // Elevator Movement
  double startElevatorTime = 0;

  // Creates the network tables object
  NetworkTableInstance inst;
  // A specific table in network tables
  NetworkTable table, gripTable;
  // A specific entry for the table
  NetworkTableEntry xEntry;
  NetworkTableEntry yEntry;

  // NewNetworkTable
  boolean angleTurnBoolean;

  NetworkTableEntry straightDistEntry, target_angleEntry, vert_distEntry, horiz_distEntry;

  double kP = 0.03;

  enum AutoMovement {
    STRAIGHT, TURN, VISION, EJECT, ELEVATOR, MODE
  }

  Object[][] autoTemplate = {
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 200, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 90, 0.5 } };

  DigitalInput elevatorLimit;
  int dpad;

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
      { AutoMovement.STRAIGHT, 200, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 90, 0.5 },
      { AutoMovement.STRAIGHT, 8, 1},
      // Activate Vision
      { AutoMovement.VISION },
      // Elevator
      { AutoMovement.ELEVATOR, ElevatorHeight.HATCH_ONE},
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, -38, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 104, 0.5 },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 225, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -14.4, 0.5 },
      { AutoMovement.STRAIGHT, 28, 1 },
      { AutoMovement.VISION },
      { AutoMovement.STRAIGHT, -173, 1 },
      { AutoMovement.TURN, 165, 0.5 },
      { AutoMovement.VISION },
      { AutoMovement.EJECT},
      // Maybe use ultrasonic distance- sensors
  };
  Object[][] sideShip2HatchLeft = {
      { AutoMovement.MODE, RobotMode.HATCH },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 222, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 90, 0.5 },
      { AutoMovement.STRAIGHT, 8, 1},
      // Activate Vision
      { AutoMovement.VISION },
      // Hatch
      { AutoMovement.ELEVATOR, ElevatorHeight.HATCH_ONE},
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, -38, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 103.4, 0.5 },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 246, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -13.4, 0.5 },
      { AutoMovement.STRAIGHT, 28, 1 },
      { AutoMovement.VISION },
      { AutoMovement.STRAIGHT, -173, 1 },
      { AutoMovement.TURN, 165, 0.5 },
      { AutoMovement.VISION },
      { AutoMovement.EJECT},
      // Maybe use ultrasonic distance- sensors
  };

  Object[][] sideShip3HatchLeft = {
      { AutoMovement.MODE, RobotMode.HATCH },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 244, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 90, 0.5 },
      { AutoMovement.STRAIGHT, 8, 1},
      // Activate Vision
      { AutoMovement.VISION },
      // Hatch
      { AutoMovement.ELEVATOR, ElevatorHeight.HATCH_ONE},
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, -38,  1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 102.3, 0.5 },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 268, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -12.4, 0.5 },
      { AutoMovement.STRAIGHT, 28, 1 },
      { AutoMovement.VISION },
      { AutoMovement.STRAIGHT, -173, 1 },
      { AutoMovement.TURN, 165, 0.5 },
      { AutoMovement.VISION },
      { AutoMovement.EJECT},

      // TODO: Figure out how to pick up a new ball
      // Maybe use ultrasonic distance- sensors
  };

  Object[][] sideShip1HatchRight = {
      { AutoMovement.MODE, RobotMode.HATCH },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 200, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -90, 0.5 },
      { AutoMovement.STRAIGHT, 8, 1},
      // Activate Vision
      { AutoMovement.VISION },      
      // Hatch
      { AutoMovement.ELEVATOR, ElevatorHeight.HATCH_ONE},
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, -38, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -104, 0.5 },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 225, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 14.4, 0.5 },
      { AutoMovement.STRAIGHT, 28, 1 },
      { AutoMovement.VISION },
      { AutoMovement.STRAIGHT, -173, 1 },
      { AutoMovement.TURN, -165, 0.5 },
      { AutoMovement.VISION },
      { AutoMovement.EJECT},
  };
  Object[][] sideShip2HatchRight = {
      { AutoMovement.MODE, RobotMode.HATCH },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 222, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -90, 0.5 },
      // Activate Vision
      { AutoMovement.STRAIGHT, 8, 1},
      { AutoMovement.VISION },
      // Hatch
      { AutoMovement.ELEVATOR, ElevatorHeight.HATCH_ONE},
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, -38, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -103.4, 0.5 },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 246, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 13.4, 0.5 },
      { AutoMovement.STRAIGHT, 28, 1 },
      { AutoMovement.VISION },
      { AutoMovement.STRAIGHT, -173, 1 },
      { AutoMovement.TURN, -165, 0.5 },
      { AutoMovement.VISION },
      { AutoMovement.EJECT},
  };

  Object[][] sideShip3HatchRight = {
      { AutoMovement.MODE, RobotMode.HATCH },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 244, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -90, 0.5 },
      { AutoMovement.STRAIGHT, 8, 1},
      // Activate Vision
      { AutoMovement.VISION },
      // Hatch
      { AutoMovement.ELEVATOR, ElevatorHeight.HATCH_ONE},
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, -38,  1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, 102.3, 0.5 },
      // Movement type, Distance, Speed
      { AutoMovement.STRAIGHT, 268, 1 },
      // Movement type, Rotation, Speed
      { AutoMovement.TURN, -12.4, 0.5 },
      { AutoMovement.STRAIGHT, 28, 1 },
      { AutoMovement.VISION },
      { AutoMovement.STRAIGHT, -173, 1 },
      { AutoMovement.TURN, 165, 0.5 },
      { AutoMovement.VISION },
      { AutoMovement.EJECT},
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
    { AutoMovement.EJECT },
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
//Automode 7 (Left Lower Rocket Hatch) Starting position is on the corner of the hab platform facing the side wall
Object[][] autoRocketHatchLeftLower = {
  {AutoMovement.MODE, RobotMode.HATCH },
  {AutoMovement.STRAIGHT, 34, 1},
  {AutoMovement.TURN, 73.5, 0.5},
  {AutoMovement.STRAIGHT, 160, 1},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_ONE},

  {AutoMovement.STRAIGHT, 22.5, -1},
  {AutoMovement.TURN, 157.5, -0.5},
  {AutoMovement.STRAIGHT, 183, 1},
  {AutoMovement.VISION},
  {AutoMovement.STRAIGHT, 10, 1},

  {AutoMovement.STRAIGHT, 183, -1},
  {AutoMovement.TURN, 157.5, 0.5},
  {AutoMovement.VISION},
  {AutoMovement.STRAIGHT, 10, 1},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_TWO}
};
/* Other Side Lower Hatch
  {AutoMovement.STRAIGHT, 94, -1},
  {AutoMovement.TURN, 20.5, -0.5},
  {AutoMovement.STRAIGHT, 211, -1},
  {AutoMovement.TURN, 32, 0.5},
  {AutoMovement.STRAIGHT, 60, 1},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_ONE},
  {AutoMovement.EJECT}
*/


//Automode 8 (Left Middle Rocket Hatch) Starting position is on the corner of the hab platform facing the side wall
Object[][] autoRocketHatchLeftMiddle = {
  {AutoMovement.MODE, RobotMode.HATCH },
  {AutoMovement.STRAIGHT, 34, 1},
  {AutoMovement.TURN, 73.5, 0.5},
  {AutoMovement.STRAIGHT, 160, 1},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_TWO},

  {AutoMovement.STRAIGHT, 22.5, -1},
  {AutoMovement.TURN, 157.5, -0.5},
  {AutoMovement.STRAIGHT, 183, 1},
  {AutoMovement.VISION},
  {AutoMovement.STRAIGHT, 10, 1},

  {AutoMovement.STRAIGHT, 183, -1},
  {AutoMovement.TURN, 157.5, 0.5},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_ONE}
};

//Automode 9 (Left Upper Rocket Hatch) Starting position is on the corner of the hab platform facing the side wall
Object[][] autoRocketHatchLeftUpper = {
  {AutoMovement.MODE, RobotMode.HATCH },
  {AutoMovement.STRAIGHT, 34, 1},
  {AutoMovement.TURN, 73.5, 0.5},
  {AutoMovement.STRAIGHT, 160, 1},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_THREE},

  {AutoMovement.STRAIGHT, 22.5, -1},
  {AutoMovement.TURN, 157.5, -0.5},
  {AutoMovement.STRAIGHT, 183, 1},
  {AutoMovement.VISION},
  {AutoMovement.STRAIGHT, 10, 1},

  {AutoMovement.STRAIGHT, 183, -1},
  {AutoMovement.TURN, 157.5, 0.5},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_ONE}
};

//Automode 10 (Right Lower Rocket Hatch) Starting position is on the corner of the hab platform facing the side wall
Object[][] autoRocketHatchRightLower = {
  {AutoMovement.MODE, RobotMode.HATCH },
  {AutoMovement.STRAIGHT, 34, 1},
  {AutoMovement.TURN, 73.5, -0.5},
  {AutoMovement.STRAIGHT, 160, 1},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_ONE},

  {AutoMovement.STRAIGHT, 22.5, -1},
  {AutoMovement.TURN, 157.5, 0.5},
  {AutoMovement.STRAIGHT, 183, 1},
  {AutoMovement.VISION},
  {AutoMovement.STRAIGHT, 10, 1},

  {AutoMovement.STRAIGHT, 183, -1},
  {AutoMovement.TURN, 157.5, -0.5},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_TWO}

};

//Automode 11 (Right Middle Rocket Hatch) Starting position is on the corner of the hab platform facing the side wall
Object[][] autoRocketHatchRightMiddle = {
  {AutoMovement.MODE, RobotMode.HATCH },
  {AutoMovement.STRAIGHT, 34, 1},
  {AutoMovement.TURN, 73.5, -0.5},
  {AutoMovement.STRAIGHT, 160, 1},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_TWO},

  {AutoMovement.STRAIGHT, 22.5, -1},
  {AutoMovement.TURN, 157.5, 0.5},
  {AutoMovement.STRAIGHT, 183, 1},
  {AutoMovement.VISION},
  {AutoMovement.STRAIGHT, 10, 1},

  {AutoMovement.STRAIGHT, 183, -1},
  {AutoMovement.TURN, 157.5, -0.5},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_ONE} 
};

//Automode 12 (Right Upper Rocket Hatch) Starting position is on the corner of the hab platform facing the side wall
Object[][] autoRocketHatchRightUpper = {
  {AutoMovement.MODE, RobotMode.HATCH },
  {AutoMovement.STRAIGHT, 34, 1},
  {AutoMovement.TURN, 73.5, -0.5},
  {AutoMovement.STRAIGHT, 160, 1},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_THREE},

  {AutoMovement.STRAIGHT, 22.5, -1},
  {AutoMovement.TURN, 157.5, 0.5},
  {AutoMovement.STRAIGHT, 183, 1},
  {AutoMovement.VISION},
  {AutoMovement.STRAIGHT, 10, 1},

  {AutoMovement.STRAIGHT, 183, -1},
  {AutoMovement.TURN, 157.5, -0.5},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_ONE} 
};

//Automode 13 (Right Middle to Upper Rocket Hatch) Starting position is on the corner of the hab platform facing the side wall
Object[][] autoRocketHatchRightMiddleUpper = {
  {AutoMovement.MODE, RobotMode.HATCH },
  {AutoMovement.STRAIGHT, 34, 1},
  {AutoMovement.TURN, 73.5, -0.5},
  {AutoMovement.STRAIGHT, 160, 1},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_TWO},

  {AutoMovement.STRAIGHT, 22.5, -1},
  {AutoMovement.TURN, 157.5, 0.5},
  {AutoMovement.STRAIGHT, 183, 1},
  {AutoMovement.VISION},
  {AutoMovement.STRAIGHT, 10, 1},

  {AutoMovement.STRAIGHT, 183, -1},
  {AutoMovement.TURN, 157.5, -0.5},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_THREE} 
};

//Automode 14 (Left Middle to Upper Rocket Hatch) Starting position is on the corner of the hab platform facing the side wall
Object[][] autoRocketHatchLeftMiddleUpper = {
  {AutoMovement.MODE, RobotMode.HATCH },
  {AutoMovement.STRAIGHT, 34, 1},
  {AutoMovement.TURN, 73.5, 0.5},
  {AutoMovement.STRAIGHT, 160, 1},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_TWO},

  {AutoMovement.STRAIGHT, 22.5, -1},
  {AutoMovement.TURN, 157.5, -0.5},
  {AutoMovement.STRAIGHT, 183, 1},
  {AutoMovement.VISION},
  {AutoMovement.STRAIGHT, 10, 1},

  {AutoMovement.STRAIGHT, 183, -1},
  {AutoMovement.TURN, 157.5, 0.5},
  {AutoMovement.VISION},
  {AutoMovement.ELEVATOR, ElevatorHeight.HATCH_THREE}
};

Object[][] visionAutoTest = {
  {AutoMovement.MODE, RobotMode.HATCH},
  {AutoMovement.VISION},
  {AutoMovement.EJECT}
};
  // Dictates the current auto that is selected
  Object[][] selectedAuto;
  // Indicates what step of auto the robot is on
  int autoStep;
  // Indicates when auto should stop
  boolean autoStop;

  enum ElevatorHeight {
    HATCH_ONE, HATCH_TWO, HATCH_THREE, BALL_ONE, BALL_TWO, BALL_THREE, BALL_SHIP, NONE
  }
  double elevatorHeight;
  ElevatorHeight destinationHeight;

  int invertControls = 1;

  boolean setStartAngle = true;
  double startAngle;

  Double straightDist, target_angle, vert_dist, horiz_dist;

  double angleTurnDelay;

  boolean hatchReturn;
  double hatchReturnTime;


  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  enum ClimbOrder { FRONT_LIFT, BACK_LIFT, FRONT_RETRACT, BACK_RETRACT }
  ClimbOrder climbOrder;
  
  @Override
  public void robotInit() {
    climbOrder = climbOrder.FRONT_LIFT;

    //UsbCamera cameraMyDude = CameraServer.getInstance().startAutomaticCapture();
    //cameraMyDude.setResolution(640, 480);

    m_chooser.setDefaultOption("No Auto", driverControlled);

    m_chooser.addOption("Left Ship Short", leftShipShort);
    m_chooser.addOption("Left Ship Middle", leftShipMiddle);
    m_chooser.addOption("Left Ship Long", leftShipLong);

    m_chooser.addOption("Right Ship Short", rightShipShort);
    m_chooser.addOption("Right Ship Middle", rightShipMiddle);
    m_chooser.addOption("Right Ship Long", rightShipLong);

    m_chooser.addOption("Left Rocket Low", leftRocketLow);
    m_chooser.addOption("Left Rocket Middle", leftRocketMedium);
    m_chooser.addOption("Left Rocket High", leftRocketHigh);

    m_chooser.addOption("Right Rocket Low", rightRocketLow);
    m_chooser.addOption("Right Rocket Middle", rightRocketMedium);
    m_chooser.addOption("Right Rocket High", rightRocketHigh);

    m_chooser.addOption("Vision Test", visionAutoTesting);

    
    m_chooser.addOption("Right Rocket Middle to High", rightRocketMiddleHigh);
    m_chooser.addOption("Left Rocket Middle to High", leftRocketMiddleHigh);

    SmartDashboard.putData("Auto choices", m_chooser);


    // Start ultrasonic sensor
    // ultrasonic = new AnalogInput(0);
    // Defines all the ports of each of the motors
    leftFront = new Spark(0);
    cameraX = new Servo(1);
    rightFront = new Spark(2);
    cameraY = new Servo(3);
    intake = new Spark(4);
    elevatorOne = new Spark(5);
    elevatorTwo = new Spark(6);
    lemmeDie = new Spark(7);
    servoOne = new Servo(8);
    servoTwo = new Servo(9);

    // Defines the left and right SpeedControllerGroups for our DifferentialDrive
    // class
    leftChassis = new SpeedControllerGroup(leftFront);
    rightChassis = new SpeedControllerGroup(rightFront);
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
    hatchSolenoid = new DoubleSolenoid(0, 2, 3);
    ballSolenoid = new DoubleSolenoid(1, 0, 1);
    frontClimb = new DoubleSolenoid(0, 4, 5);
    backClimb = new DoubleSolenoid(0, 6, 7);
    hatchGrabSolenoid = new DoubleSolenoid(0, 0, 1);
    intakeSolenoid = new DoubleSolenoid(1, 2, 3);

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
    gripTable = inst.getTable("GRIP");
    // Get X and Y entries
    //xEntry = table.getEntry("xEntry");
    xEntry = gripTable.getEntry("myContoursReport/centerX");
    yEntry = table.getEntry("yEntry");

    straightDistEntry = SmartDashboard.getEntry("straightDist");
    target_angleEntry = SmartDashboard.getEntry("target_angle");
    vert_distEntry = SmartDashboard.getEntry("vert_dist");
    horiz_distEntry = SmartDashboard.getEntry("horiz_dist");

    // when enabled, the PCM will turn on the compressor when the pressure switch is
    // closed
    c.setClosedLoopControl(true);// or false

    // set the state of the valve
    ballSolenoid.set(DoubleSolenoid.Value.kOff);
    hatchSolenoid.set(DoubleSolenoid.Value.kOff);
    frontClimb.set(DoubleSolenoid.Value.kOff);
    backClimb.set(DoubleSolenoid.Value.kOff);
    hatchGrabSolenoid.set(DoubleSolenoid.Value.kOff);
    intakeSolenoid.set(DoubleSolenoid.Value.kOff);

    // Maps a joystick to a variable
    driver = new Joystick(0);
    destinationHeight = ElevatorHeight.NONE;

    elevatorLimit = new DigitalInput(6);
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
    horiz_dist = horiz_distEntry.getDouble(0);
    vert_dist = vert_distEntry.getDouble(0);
    target_angle = target_angleEntry.getDouble(0);
    straightDist = straightDistEntry.getDouble(0);
    //System.out.println(straightDist);
    //System.out.println(target_angle);
    //System.out.println(vert_dist);
    //System.out.println(horiz_dist);
    
        //System.out.println(operator.getRawAxis(5));
    // Main Robot Movement
    chassis.arcadeDrive(-driver.getX(), (invertControls*driver.getY()));

    // Driver Input Buttons
    //aButton = driver.getRawButton(1);
    aButtonPressed = driver.getRawButtonPressed(1);
    //bButton = driver.getRawButton(2);
    bButtonPressed = driver.getRawButtonPressed(2);
    xButton = driver.getRawButton(3);
    yButton = driver.getRawButtonPressed(4);
    lBumper = driver.getRawButton(5);
		rBumper = driver.getRawButton(6);
		select = driver.getRawButton(7);
		start = driver.getRawButton(8);
		leftThumbPush = driver.getRawButton(9);
    rightThumbPush = driver.getRawButton(10);

    if (start) {
      autoStop = true;
    }
    if (lBumper){
      angleTurnBoolean = !angleTurnBoolean;
    } 
    if (rBumper) {
      invertControls = -1;
    } else {
      invertControls = 1;
    }

    if (xButton) {
      switch (climbOrder){
        case FRONT_LIFT:
          frontClimb.set(DoubleSolenoid.Value.kForward);
          climbOrder = ClimbOrder.BACK_LIFT;
          break;
        case BACK_LIFT:
          backClimb.set(DoubleSolenoid.Value.kForward);
          climbOrder = ClimbOrder.FRONT_RETRACT;
          break;
        case FRONT_RETRACT:
          frontClimb.set(DoubleSolenoid.Value.kReverse);
          climbOrder = ClimbOrder.BACK_RETRACT;
          break;
        case BACK_RETRACT:
          backClimb.set(DoubleSolenoid.Value.kReverse);
          climbOrder = ClimbOrder.FRONT_LIFT;
          break;
      }
    }

    if (yButton) {
      backClimb.set(DoubleSolenoid.Value.kReverse);
      frontClimb.set(DoubleSolenoid.Value.kReverse);
      climbOrder = ClimbOrder.FRONT_LIFT;
    }
/*
    if (driver.getRawAxis(4) > 0.2 || driver.getRawAxis(4) < -0.2){
      
    }
    if (driver.getRawAxis(5) > 0.2 || driver.getRawAxis(5) < -0.2){
      
    }*/
    cameraX.setAngle(driver.getRawAxis(4)*90 + 90);
    cameraY.setAngle(-driver.getRawAxis(5)*90 + 90);

    //System.out.println(cameraX.getAngle());
    //System.out.println(cameraY.getAngle());

    /*
    if (!climbFrontTimer.isActive) {
      if (aButton) {
        if (frontClimb.get() == DoubleSolenoid.Value.kReverse){
          frontClimb.set(DoubleSolenoid.Value.kForward);
        } else {
          frontClimb.set(DoubleSolenoid.Value.kReverse);
        }
      }
      climbFrontTimer.activate();
    }

    if (!climbBackTimer.isActive) {
      if (bButton) {
        if (backClimb.get() == DoubleSolenoid.Value.kReverse){
          backClimb.set(DoubleSolenoid.Value.kForward);
        } else {
          backClimb.set(DoubleSolenoid.Value.kReverse);
        }
      }
      climbBackTimer.activate();
    }*/
    
    if (aButtonPressed) {
      if (frontClimb.get() == DoubleSolenoid.Value.kReverse){
        frontClimb.set(DoubleSolenoid.Value.kForward);
      } else {
        frontClimb.set(DoubleSolenoid.Value.kReverse);
      }
    }

    if (bButtonPressed) {
      if (backClimb.get() == DoubleSolenoid.Value.kReverse){
        backClimb.set(DoubleSolenoid.Value.kForward);
      } else {
        backClimb.set(DoubleSolenoid.Value.kReverse);
      }
    }

    // OPERATOR CONTROLS BEGINS
    aButtonOp = operator.getRawButtonPressed(1);
    bButtonOp = operator.getRawButtonPressed(2);
    xButtonOp = operator.getRawButtonPressed(3);
    yButtonOp = operator.getRawButtonPressed(4);
    lBumperOp = operator.getRawButtonPressed(5);
    rBumperOpPressed = operator.getRawButtonPressed(6);
		selectOp = operator.getRawButtonPressed(7);
		startOp = operator.getRawButtonPressed(8);
		leftThumbPushOp = operator.getRawButton(9);
    rightThumbPushOp = operator.getRawButton(10);
    dpad = operator.getPOV();

    if (10 < dpad && dpad < 180) {
      hatchReturn = true;
      hatchReturnTime = System.currentTimeMillis();
    } else if (dpad >= 180) {
      if (hatchSolenoid.get() == DoubleSolenoid.Value.kReverse){
        hatchSolenoid.set(DoubleSolenoid.Value.kForward);
      } else {
        hatchSolenoid.set(DoubleSolenoid.Value.kReverse);
      }
    }

    if (startOp) {
      if (hatchGrabSolenoid.get() == DoubleSolenoid.Value.kReverse){
        hatchGrabSolenoid.set(DoubleSolenoid.Value.kForward);
      } else {
        hatchGrabSolenoid.set(DoubleSolenoid.Value.kReverse);
      }
    }

    if (selectOp) {
      operatorOverride = !operatorOverride;
      destinationHeight = ElevatorHeight.NONE;
      startElevatorTime = 0;
    }

    elevator.set(operator.getRawAxis(5));

    //lemmeDie.set(-operator.getY());
    if (operator.getY() > 0.2){
      intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    if (operator.getY() < -0.2) {
      intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    } 

    if (operator.getRawAxis(3) > 0.1) {
      intake.set(operator.getRawAxis(3));
    } else if (operator.getRawAxis(2) > 0.1) {
      intake.set(-operator.getRawAxis(2));
    } else {
      intake.set(0);
    }

    if (rBumperOpPressed) {
      switchMode();
    }

    if (lBumperOp) {
      doFire = true;
    }

    /*
    if (aButtonOp) {
      if (hatchSolenoid.get() == DoubleSolenoid.Value.kReverse){
        hatchSolenoid.set(DoubleSolenoid.Value.kForward);
      } else {
        hatchSolenoid.set(DoubleSolenoid.Value.kReverse);
      }
    }
    if (bButtonOp) {
      if (ballSolenoid.get() == DoubleSolenoid.Value.kReverse){
        ballSolenoid.set(DoubleSolenoid.Value.kForward);
      } else {
        ballSolenoid.set(DoubleSolenoid.Value.kReverse);
      }
    }
    if (xButtonOp) {
      if (intakeSolenoid.get() == DoubleSolenoid.Value.kReverse){
        intakeSolenoid.set(DoubleSolenoid.Value.kForward);
      } else {
        intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
      }
    }
    if (yButtonOp) {
      if (hatchGrabSolenoid.get() == DoubleSolenoid.Value.kReverse){
        hatchGrabSolenoid.set(DoubleSolenoid.Value.kForward);
      } else {
        hatchGrabSolenoid.set(DoubleSolenoid.Value.kReverse);
      }
    }*/

    if (currentRobotMode == RobotMode.CARGO) {
      if (aButtonOp) {
        destinationHeight = ElevatorHeight.BALL_ONE;
        //System.out.println("Got to A");
      }
      if (bButtonOp) {
        destinationHeight = ElevatorHeight.BALL_TWO;
        //System.out.println("Got to B");
      }
      if (yButtonOp) {
        destinationHeight = ElevatorHeight.BALL_THREE;
      }
      if (xButtonOp) {
        destinationHeight = ElevatorHeight.BALL_SHIP;
      }
    } else {
      if (aButtonOp) {
        destinationHeight = ElevatorHeight.HATCH_ONE;
      }
      if (bButtonOp) {
        destinationHeight = ElevatorHeight.HATCH_TWO;
      }
      if (yButtonOp) {
        destinationHeight = ElevatorHeight.HATCH_THREE;
      }
    }



    // print compressor status to the console
    // //System.out.println(enabled + "/n" + pressureSwitch + "/n" + current);

    // //System.out.println("Im In");
    updateSmartDashboard();

    if (doFire) {
      fire();
    }

    //System.out.println("getElevatorHeight(): " + getElevatorHeight());
    //System.out.println("Encoder Counts: " + elevatorEncoder.get());
    //System.out.println(destinationHeight);

    if (elevatorLimit.get()){
      elevatorEncoder.reset();
    }

    if (destinationHeight != ElevatorHeight.NONE) {
      //System.out.println("got to Robot Periodic");
      if (elevatorMovement(destinationHeight)) {
        //System.out.println("elevatorMovement returned true");
        elevator.set(0.2);
        destinationHeight = ElevatorHeight.NONE;
        doFire = true;
        startElevatorTime = System.currentTimeMillis();
      }
      else {
        //System.out.println("Kill Me Now");
      }
    }

    if (startElevatorTime + 600 < System.currentTimeMillis() && startElevatorTime > 1) {
      //System.out.println("lowering elevator");
      if (elevatorDown()) {
        startElevatorTime = 0;
        autoStep++;
        elevatorEncoder.reset();
        elevator.set(0);
      }
    }

    if (getElevatorHeight() >= 65) {
      elevator.set(0);
    }

    if (angleTurnBoolean) {
      if (angleTurn(target_angle, 0.5)){
        //if (System.currentTimeMillis() - angleTurnDelay > 2000){
          if (angleDrive(straightDist, 0.4)) {
            angleTurnBoolean = false;
          }
        //}
      }// else {
        //angleTurnDelay = System.currentTimeMillis();
      //}
    }

    //System.out.println(elevatorLimit.get());
    if (hatchReturn) {
      hatchGrabSolenoid.set(DoubleSolenoid.Value.kReverse);
      if (hatchReturnTime + 200 < System.currentTimeMillis()) {
        hatchSolenoid.set(DoubleSolenoid.Value.kForward);
        hatchReturn = false;
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
    // Has the auto finished?
    autoStop = false;

    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    //System.out.println("Auto selected: " + m_autoSelected);

    // The step that auto is on
    autoStep = 0;
/*
    m_chooser.addOption("Right Ship Short", rightShipShort);
    m_chooser.addOption("Right Ship Middle", rightShipMiddle);
    m_chooser.addOption("Right Ship Long", rightShipLong);

    m_chooser.addOption("Left Rocket Low", leftRocketLow);
    m_chooser.addOption("Left Rocket Middle", leftRocketMedium);
    m_chooser.addOption("Left Rocket High", leftRocketHigh);

    m_chooser.addOption("Right Rocket Low", rightRocketLow);
    m_chooser.addOption("Right Rocket Middle", rightRocketMedium);
    m_chooser.addOption("Right Rocket High", rightRocketHigh);

    m_chooser.addOption("Vision Test", visionAutoTesting);
    m_chooser.addOption("No Auto", driverControlled);

    SmartDashboard.putData("Auto choices", m_chooser);
    m_autoSelected = (String) m_chooser.getSelected(); */


    selectedAuto = autoTemplate;
    // Which auto are we using?
    switch (m_autoSelected) {
      case "Left Ship Short":
        selectedAuto = sideShip1HatchLeft;
        break;
      case "Left Ship Middle":
        selectedAuto = sideShip2HatchLeft;
        break;
      case "Left Ship Long":
        selectedAuto = sideShip3HatchLeft;
        break;
      case "Right Ship Short":
        selectedAuto = sideShip1HatchRight;
        break;
      case "Right Ship Middle":
        selectedAuto = sideShip2HatchRight;
        break;
      case "Right Ship Long":
        selectedAuto = sideShip3HatchRight;
        break;
      case "Left Rocket Low":
        selectedAuto = autoRocketHatchLeftLower;
        break;
      case "Left Rocket Middle":
        selectedAuto = autoRocketHatchLeftMiddle;
        break;
      case "Left Rocket High":
        selectedAuto = autoRocketHatchLeftUpper;
        break;
      case "Right Rocket Low":
        selectedAuto = autoRocketHatchRightLower;
        break;
      case "Right Rocket Middle":
        selectedAuto = autoRocketHatchRightMiddle;
        break;
      case "Right Rocket High":
        selectedAuto = autoRocketHatchRightUpper;
        break;
      case visionAutoTesting:
        selectedAuto = visionAutoTest;
      case "Right Rocket Middle to High":
        selectedAuto = autoRocketHatchRightMiddleUpper;
        break;
      case "Left Rocket Middle to High":
        selectedAuto = autoRocketHatchLeftMiddleUpper;
        break;
      case driverControlled:
        autoStop = true;
        break;
      default:
        selectedAuto = autoRocketHatchRightUpper;
      
    }
    System.out.println(selectedAuto);
    

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
          if (currentRobotMode == RobotMode.CARGO) {
            switchMode();
            System.out.println("switchMode()");
          }
          System.out.println("AutoMovement.MODE");
          this.autoStep++;
          System.out.println(autoStep);
          break;
        case STRAIGHT:
          System.out.println("AutoMovement.STRAIGHT");
          if (getDistance() < ((int) selectedAuto[autoStep][1]) - 10) { // Forwards
            leftChassis.set((int) selectedAuto[autoStep][2] - 10);
            rightChassis.set((int) selectedAuto[autoStep][2] - 10);
          } else if (getDistance() > (int) selectedAuto[autoStep][1] + 10) { // Backwards
            leftChassis.set(-(int) selectedAuto[autoStep][2] - 10);
            rightChassis.set(-(int) selectedAuto[autoStep][2] - 10);
          } else { // Destination Reached
            resetEncoders();
            System.out.println("AutoMovement.STRAIGHT Complete");
            autoStep++;
          }
          break;
        case TURN:
          System.out.println("AutoMovement.TURN");
          if (angleTurn((double) selectedAuto[autoStep][1], (double) selectedAuto[autoStep][2])) {
            resetEncoders();
            System.out.println("AutoMovement.TURN Complete");
            autoStep++;
          }
          break;
        case ELEVATOR:
          destinationHeight = (ElevatorHeight) selectedAuto[autoStep][1];
          System.out.println("AutoMovement.ELEVATOR");
          break;
        case EJECT:
          //doFire = true;
          break;
        case VISION:
          if (cameraControl()) {
            System.out.println("AutoMovement.VISION");
            break;
          } else {
            // TODO: Add a coninuation section for auto code. COMPLETED, NEEDS TESTING
            System.out.println("AutoMovement.VISION complete");
            autoStep++;
          }
          break;
      }
    } else {
      
    }

  }

  /**
   * This function is called as teleop is Initiated
   */
  @Override
  public void teleopInit() {
    super.teleopInit();
    elevatorEncoder.reset();
    operatorOverride = false;
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {


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
     * //System.out.println("Turning Left " + xEntry.getDouble(middlePixel)); // turn
     * left } else if (xEntry.getDouble(0.0) > middlePixel - threshold) {
     * //System.out.println("Turning Right " + xEntry.getDouble(middlePixel)); // turn
     * right } else { //System.out.println("Driving Straight " +
     * xEntry.getDouble(middlePixel)); // drive straight }
     */
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledInit() {
    angleTurnBoolean = false;
  }

  // Converts encoder counts into inches
  public double getDistance() {
    return ((double) (leftEncoder.get() + rightEncoder.get()) / (ENCODER_COUNTS_PER_INCH * 2));
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
  public boolean cameraControl() {
    // Sets the threshold for vision
    int threshold = 15;
    double[] xDefault= {middlePixel, middlePixel};

    double[] xArray = xEntry.getDoubleArray(xDefault);
    double xBuffer = 0;

    for (int xLoop = 0; xLoop < xArray.length; xLoop++){
      xBuffer += xArray[xLoop];
    }
    
    //double xValue = xBuffer/xArray.length;
    double xValue = target_angleEntry.getDouble(320);

    if (xValue < middlePixel + threshold) {
      leftChassis.set(-0.2);
      rightChassis.set(-0.4);
      System.out.println("Turning Left " + xValue);
      // turn left
      
    } else if (xValue > middlePixel - threshold) {
      leftChassis.set(-0.4);
      rightChassis.set(-0.2);
      System.out.println("Turning Right " + xValue);
      // turn right      
    } 
    else {
      //leftChassis.set(0.5);
      //rightChassis.set(0.5);
      System.out.println("Driving Straight " + xValue);
      // drive straight  
       
    }
    return true;
    /*
    if ((xValue - oldX) < 0.01) {
      return false;        
    } else {
      oldX = xEntry.getDouble(0.0);
      return true;
    }*/     
    
    

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
     * //System.out.println(this.leftDistance); }
     * 
     * if (ultraMiddle.getRangeInches() < ultraRight.getRangeInches()) {
     * this.rightDistance = ultraRight.getRangeInches();
     * //System.out.println(this.rightDistance); }
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
    
    //SmartDashboard.putNumber("ServoOne", servoOne.getAngle());
    //SmartDashboard.putNumber("ServoTwo", servoTwo.getAngle());

    SmartDashboard.putNumber("Robot Speed", robotSpeed());

    String xEnt = xEntry.toString();
    SmartDashboard.putString("X Entry", xEnt);

    SmartDashboard.putNumber("Elevator Encoders", elevatorEncoder.get());
    SmartDashboard.putNumber("Elevator Height", getElevatorHeight());

    SmartDashboard.putString("Current Robot mode", currentRobotMode.name());
    SmartDashboard.putBoolean("elevatorLimit", elevatorLimit.get());

    //SmartDashboard.putNumber("Ultrasonic Distance", getUltrasonicDistance());
  }
  /*
  public double getUltrasonicDistance() {
    return (double) (((ultrasonic.getAverageVoltage() * 1000) / 238.095) + 9.0);
  }*/

  public boolean elevatorMovement(ElevatorHeight level) {
    double firstHeight = 0;
    switch (level) {
      case HATCH_ONE:
        elevatorHeight = firstHeight + 0.1;
        break;
      case HATCH_TWO:
        elevatorHeight = firstHeight + 22;
        break;
      case HATCH_THREE:
        elevatorHeight = firstHeight + 47;
        break;
      case BALL_ONE:
        elevatorHeight = firstHeight + 2.25;
        break;
      case BALL_TWO:
        elevatorHeight = firstHeight + 23.25;
        break;
      case BALL_THREE:
        elevatorHeight = firstHeight + 47;
        break;
      case BALL_SHIP:
        elevatorHeight = firstHeight + 8.25;
      default:
        break;
    }

    //System.out.println("getElevatorHeight(): " + getElevatorHeight());
    //System.out.println("elevatorHeight: " + elevatorHeight);

    if (getElevatorHeight() <= elevatorHeight) {
      //elevator.set(((0-1)/((0-(elevatorHeight/2))*(0-(elevatorHeight/2))))*((getElevatorHeight()-(elevatorHeight/2))*(getElevatorHeight()-(elevatorHeight/2))) + 1);
      elevator.set(0.7);
      //System.out.println("One");
      //System.out.println(getElevatorHeight());
    }
    

    if (getElevatorHeight() >= elevatorHeight) {
      return true;
    } else {
      return false;
    }
  }

  public boolean elevatorDown() {
    elevatorHeight = 0.5;
    if (!elevatorLimit.get()) {
      //elevator.set(-(((0-1)/((0-(elevatorHeight/2))*(0-(elevatorHeight/2))))*((getElevatorHeight()-(elevatorHeight/2))*(getElevatorHeight()-(elevatorHeight/2))) + 1));
      elevator.set(-0.2);
      //System.out.println("Negative");
    }
    
    if  (elevatorLimit.get()) {
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
    ////System.out.println(answer);
    return answer;
  }

  public void switchMode() {
    if (currentRobotMode == RobotMode.HATCH) {
      currentRobotMode = RobotMode.CARGO;
      servoClose();
      //System.out.println("We are in Cargo Mode");
    } else {
      currentRobotMode = RobotMode.HATCH;
      servoOpen();
      //System.out.println("We are in Hatch Mode");
    }
  }

  public void startPistonTimer() {
    startPistonTime = System.currentTimeMillis();
  }

  /*
  public void ultrasonicStop() {
    if (getUltrasonicDistance() <= 1) {
      chassis.stopMotor();
    }
  }*/

  public void fire() {
    //System.out.println("we are fire()ing");
    if (startStartPistonTime) {
        startPistonTimer();
        startStartPistonTime = false;
    }
    if (currentRobotMode == RobotMode.HATCH) {
      if (System.currentTimeMillis() < this.startPistonTime + 200) {
        c.setClosedLoopControl(false);
        hatchSolenoid.set(DoubleSolenoid.Value.kReverse);
      } else if (System.currentTimeMillis() < this.startPistonTime + 900) {
        hatchGrabSolenoid.set(DoubleSolenoid.Value.kForward);
      } else {
        hatchSolenoid.set(DoubleSolenoid.Value.kForward);
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

  public boolean angleTurn(double turnAngle, double turnSpeed) {
    System.out.println("angleTurn()");
    if (setStartAngle) {
      startAngle = getAngle();
      setStartAngle = false;
    }
    if (getAngle() - startAngle < (turnAngle - 3)) {
      leftChassis.set(-turnSpeed);
      rightChassis.set(turnSpeed);
      return false;
    } else if (getAngle() - startAngle > (turnAngle + 3)) {
      leftChassis.set(turnSpeed);
      rightChassis.set(-turnSpeed);
      return false;
    } else { // Turn Complete
      setStartAngle = true;
      return true;
    }
  }

  public boolean angleDrive(double distance, double speed) {
    if (distance > 27) {
      leftChassis.set(-speed);
      rightChassis.set(-speed);
      return false;
    } else {
      return true;
    }
  }

  private void driveStraight(double heading, double speed) {
		
		// get the current heading and calculate a heading error
		double currentAngle = gyro.getAngle()%360.0;
		
		double error = heading - currentAngle;

		// calculate the speed for the motors
		double leftSpeed = speed;
		double rightSpeed = speed;

		// adjust the motor speed based on the compass error
		if (error < 0) {
			// turn left
			// slow down the left motors
			leftSpeed += error * kP;
		}
		else {
			// turn right
			// Slow down right motors
			rightSpeed -= error * kP;
		}
		
		// set the motors based on the inputted speed
    leftChassis.set(-leftSpeed);
    rightChassis.set(-rightSpeed);
	}
  
  /*
  public void threadedStraight(){
    Thread t = new Thread(() -> {
      while (RobotState.isEnabled() && !Thread.interrupted()) {
    // Not stuck anymore!
        leftChassis.set(0.2);
        rightChassis.set(0.2);
      }
    });
    t.start();
  }*/
  
}