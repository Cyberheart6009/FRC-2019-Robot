/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Add your docs here.
 */
public class AutoRunner {

  DifferentialDrive chassis;
  
  AutoMovement moveType;
  
  double special;

  double speed;

  boolean didSetCurrentValue;
  double currentValue;

  public AutoRunner(DifferentialDrive chassis){
    this.chassis = chassis;
  }

  public boolean move() {
    if (moveType == AutoMovement.STRAIGHT) {
    } else if (moveType == AutoMovement.TURN){

    }
  return true;
  }
}

enum AutoMovement {
  STRAIGHT,
  TURN
}