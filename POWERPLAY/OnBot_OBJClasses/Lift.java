package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.JavaUtil;


public class Lift extends BlocksOpModeCompanion {

  static AnalogInput liftVoltage;
  static Servo servoLift;

  // Java class variables for detecting & controlling lift range
  static double voltageLowest = 0.45;  // 0.45  // static variables defined across java class
  static double voltageHighest = 1.19;  // 1.19  static variables defined across java class
  
  // Initialize our junction lift levels
  // liftDropLevel: Range voltageLowest - voltageHighest
  static double[] liftJunctionLevel = {0.455, 0.84, 1.189};  // {0.45, 0.84, 1.14}

  // Initialize our pickup lift levels
  // Range voltageLowest - voltageHighest
  static double[] liftPickupLevel = {0.455, 0.50, 0.56};  // {0.43, 0.47, 0.52, 0.55, 0.58}

  // Determines whether the lift is in an event driven mode of moving to a target
  static boolean liftMovingToTarget = false;
  static double targetLevel = voltageLowest;  //  Target level by holding button
  static double targetThreshold = 0.035;
  
  // Will be current reading for our analog sensor associated with lift
  // ***Values will vary if the cord and springs are stretched***
  static double currentVoltage;

  // Prototyped values to understand range of lift for scaled movement
  static double liftPowerToMoveUp = 115;
  static double liftPowerToMoveDown = 180;
  static double liftPowerForNoMovement = 150;
  static double liftPowerRange = 100;
  static double liftPowerMin = 100;
    
  // Java class variables for movement of the lift motor
  static double liftPowerScaledNoMovement = (liftPowerForNoMovement - liftPowerMin) / liftPowerRange;
  static double liftPowerScaledMoveUp = (liftPowerToMoveUp - liftPowerMin) / liftPowerRange;;
  static double liftPowerScaledMoveDown = (liftPowerToMoveDown - liftPowerMin) / liftPowerRange;


  @ExportToBlocks (
    heading = "Initialize Lift",
    color = 255,
    comment = "Initialization of lift and variables.",
    tooltip = "Initialization of lift and variables.",
    parameterLabels = {"Lift Motor Name", "Lift Analog Sensor Name"}
  )
  /**
   * Initialization of lift and variables
   */
  public static void initLift(String liftMotorName, String liftAnalogSensorName) {

    // initialize our handles on hardware parts of our lift
    // String names provided to this function should be the same as those in active hardware configuration
    servoLift = hardwareMap.get(Servo.class, liftMotorName);
    liftVoltage = hardwareMap.get(AnalogInput.class, liftAnalogSensorName);
 
    telemetry.addData("Lowest voltage", Double.parseDouble(JavaUtil.formatNumber(voltageLowest, 2)));
    telemetry.addData("Highest voltage", Double.parseDouble(JavaUtil.formatNumber(voltageHighest, 2)));
    telemetry.addData("Scaled Power to Move Lift Up:", Double.parseDouble(JavaUtil.formatNumber(liftPowerScaledMoveUp, 2)));
    telemetry.addData("Scaled Power to Move Lift Down:", Double.parseDouble(JavaUtil.formatNumber(liftPowerScaledMoveDown, 2)));
    telemetry.addData("Scaled Power for No Movement:", Double.parseDouble(JavaUtil.formatNumber(liftPowerScaledNoMovement, 2)));
  }  // end method initLift()
  
  @ExportToBlocks (
    heading = "Stop Lift Movement",
    color = 255,
    comment = "Stop lift movement.",
    tooltip = "Stop lift movement."
  )
  /**
   * Stop movement of the lift
   */
  public static double stopLiftMovement() {
    
    // Make sure our event driven routines to lift to a target level will also stop
    liftMovingToTarget = false;

    // Get our lastest voltage (aka lift position)
    currentVoltage = liftVoltage.getVoltage();

    // Default target is current position     
    targetLevel = currentVoltage;

    telemetry.addData("Lift Power for NO Movement:", liftPowerScaledNoMovement);
    
    servoLift.setPosition(liftPowerScaledNoMovement);

    return currentVoltage;
    
  }  // end method stopLiftMovement()

  @ExportToBlocks (
    heading = "Determine Whether Moving To Target Lift Level",
    color = 255,
    comment = "Returns whether we are moving to a target lift level",
    tooltip = "Returns whether we are moving to a target lift level"
  )
  /**
   * Returns whether we are moving to a target lift level
   */
  public static boolean movingToTarget() {
    return liftMovingToTarget;
  }  // end method movingToTarget()
  
  @ExportToBlocks (
    heading = "Have We Reached Target Lift Level",
    color = 255,
    comment = "This method is called in the running OpsMode AFTER calling moveToLevelSpecified",
    tooltip = "To stop this action you need to manually raise or lower the lift."
  )
  /**
   * Follow-up to check if we have reached our target lift level
   * Should check to make sure not reaching high or low limit
   */
  public static boolean targetLevelReached() {
    
    // if already out of state then exit
    if ( !liftMovingToTarget )
      return liftMovingToTarget;
      
    // Get our lastest voltage (aka lift position)
    currentVoltage = liftVoltage.getVoltage();
    
    // See if we are within acceptable range of our target level
    if (Math.abs(currentVoltage - targetLevel) < targetThreshold) {
      stopLiftMovement();
      liftMovingToTarget = false;
      return liftMovingToTarget;
    }

    // Stop if we are close to our limits    
    if ((currentVoltage <= voltageLowest) || (currentVoltage >= voltageHighest)) {
      stopLiftMovement();
      liftMovingToTarget = false;
      return liftMovingToTarget;
    }

    return liftMovingToTarget;
    
  }  // end method targetLevelReached()
  
  @ExportToBlocks (
    heading = "Move To Level Specified",
    color = 255,
    comment = "Move to level[0,1,2] specified by button selected[A, B/X, Y] and whether grabber is closed.",
    tooltip = "Move to a level. Send -1 to check status if ready to stop.",
    parameterLabels = {"Level Request", "Is Grabber Closed"}
  )
  /**
   * Initiate a move of the lift to a specified level 
   * Will need to either move up or down
   * Should check to make sure not reaching high or low limit
   * ==> NEED TO CALL targetLevelReached() in the calling program of this method to stop the action <==
   */
  public static boolean moveToLevelSpecified(int levelRequested, boolean grabberClosed) {
    
    // invalid index to an array
    if (levelRequested < 0) {
      stopLiftMovement();
      liftMovingToTarget = false;
      return liftMovingToTarget;
    }

    // Determine which level [Junction or Pickup] is being requesting
    // Based off of whether the Grabber is closed
    if (grabberClosed) {
      // Grabber closed so assuming the level requested is a Junction level
      
      // if an invalid index to the Junction levels, then just return and don't do anything
      if (levelRequested > liftJunctionLevel.length - 1) {
        stopLiftMovement();
        liftMovingToTarget = false;
        return liftMovingToTarget;
      }
      
      // Set our target voltage level to the level requested  
      targetLevel = liftJunctionLevel[levelRequested];
      
    } else {
      // Grabber open so assuming the level requested is a pickup level
      
      // if an invalid index to the pickup levels, then just return and don't do anything
      if (levelRequested > liftPickupLevel.length - 1) {
        stopLiftMovement();
        liftMovingToTarget = false;
        return liftMovingToTarget;
      }
      
      // Set our target voltage level to the level requested  
      targetLevel = liftPickupLevel[levelRequested];
      
    }
    
    // if target level is past our maximum limit then return and don't do anything
    if (targetLevel >= voltageHighest ) {
      stopLiftMovement();
      liftMovingToTarget = false;
      return liftMovingToTarget;
    // if target level is past our low limit then return and don't do anything
    } else if (targetLevel <= voltageLowest) {
      stopLiftMovement();
      liftMovingToTarget = false;
      return liftMovingToTarget;
    }

    // Set information to be displayed
    telemetry.addData("Lift TARGET LEVEL: ", targetLevel);
    telemetry.addData("Lift CURRENT VOLTAGE: ", currentVoltage);
    telemetry.addData("Lift Power To Move Up: ", liftPowerScaledMoveUp);
    telemetry.addData("Lift Power To Move Down: ", liftPowerScaledMoveDown );
    telemetry.addData("Lift MAX: ", voltageHighest);
    telemetry.addData("Lift LOW: ", voltageLowest);

    telemetry.update();

    // Get our lastest voltage (aka lift position)
    currentVoltage = liftVoltage.getVoltage();
    
    // If target level is higher than our current level then initiate moving up
    if (isTargetLevelHigher(targetLevel)) {
      // Move up until able to reach our targetDropLevel or Highest limit
      if (currentVoltage < voltageHighest && targetLevel > currentVoltage) {

        // Initiate moving up to target
        servoLift.setPosition(liftPowerScaledMoveUp);
        
        currentVoltage = liftVoltage.getVoltage();
        telemetry.addData("Lift Target Level: ", targetLevel);
        telemetry.addData("Lift Current Voltage: ", currentVoltage);

        // Save our event driven state that we are moving to a target
        liftMovingToTarget = true;
        
      } else {
        stopLiftMovement();
        liftMovingToTarget = false;
      }
    
    // If target level is lower than our current level then initiate moving down
    } else {
      // Move down until able to reach our targetLevel or lowest limit
      if (currentVoltage > voltageLowest && targetLevel < currentVoltage) {

        // Initiate moving down to target
        servoLift.setPosition(liftPowerScaledMoveDown);
        
        currentVoltage = liftVoltage.getVoltage();
        telemetry.addData("Lift Target Level: ", targetLevel);
        telemetry.addData("Lift Current Voltage: ", currentVoltage);

        // Save our event driven state that we are moving to a target
        liftMovingToTarget = true;
        
      } else {
        stopLiftMovement();
        liftMovingToTarget = false;
      }
      
    }
  
    return liftMovingToTarget;
    
  }  // end method moveToNextHigherDropLevelVoltage()

  private static boolean isTargetLevelHigher(double targetVoltage) {
    /*   Returns the following:
     *        TRUE  Target is greater than current voltage
     *        FALSE Target is not higher than to current voltage
     */

    // Get our lastest voltage (aka lift position)
    currentVoltage = liftVoltage.getVoltage();
    
    if (targetVoltage > currentVoltage)
      return true;
    else
      return false;
    
  }  // end method isTargetLevelHigher()

  private static double findNextHigherJunctionLevelVoltage() {
    /*   Returns the following:
     *        -1  Higher level is greater than lift maximum height limit
     *         0  No higher Junction level compared to current voltage
     *   +double  Voltage of Junction level higher than current voltage
     */

    // Get our lastest voltage (aka lift position) while also stopping lift movement
    currentVoltage = stopLiftMovement();
    
    for (int i = 0; i < liftJunctionLevel.length; i++) {
      if (currentVoltage < liftJunctionLevel[i]) {
        // Found a higher Junction level
        targetLevel = liftJunctionLevel[i];
        break;
      }
    }
    
    // If targetJunctionLevel is higher than our highest limit then don't go to it
    if (targetLevel > voltageHighest)
      return -1;
  
    return targetLevel;
    
  }  // end method findNextHigherJunctionLevelVoltage()


  private static double findNextLowerJunctionLevelVoltage() {
    /*   Returns the following:
     *        -1  Lower level is lower than lift lowest height limit
     *         0  No lower Junction level compared to current voltage
     *   +double  Voltage of junction level lower than current voltage
     *   ASSUMPTION: Voltage is always positive.
     */

    // Get our lastest voltage (aka lift position) while also stopping lift movement
    currentVoltage = stopLiftMovement();
    
    for (int i = liftJunctionLevel.length-1; i >= 0; i--) {
      if (currentVoltage > liftJunctionLevel[i]) {
        // Found a lower drop level
        targetLevel = liftJunctionLevel[i];
        break;
      }
    }
    
    // If targetDropLevel is lower than our lowest limit then don't go to it
    if (targetLevel < voltageLowest )
      return -1;
  
    return targetLevel;
    
  }  // end method findNextLowerJunctionLevelVoltage()
  
  @ExportToBlocks (
    heading = "Move To Next Higher Junction Level",
    color = 255,
    comment = "Move to next higher Junction level.",
    tooltip = "Move to next higher Junction level."
  )
  /**
   * Move lift up one Junction level 
   * Should check to make sure not reaching high limit
   */
  public static double moveToNextHigherJunctionLevelVoltage() {
    
    // Get our lastest voltage (aka lift position) while also stopping lift movement
    currentVoltage = stopLiftMovement();
    
    targetLevel = findNextHigherJunctionLevelVoltage();
    
    // Set information to be displayed
    telemetry.addData("Lift Request:", "Move To Next Higher Level" );
    telemetry.addData("Lift Target Junction Level: ", targetLevel);
    telemetry.addData("Lift Max Limit: ", currentVoltage);
    telemetry.addData("Lift Power To Move Up: ", liftPowerScaledMoveUp);
    telemetry.addData("Lift Max Limit: ", voltageHighest);

    telemetry.update();
  
    // if targetJunctionLevel is -1 or 0 then we don't move up
    // will return the current voltage and get out of our method
    if (targetLevel <= 0.0)
      return currentVoltage;
      
    // Move up until able to reach our targetDropLevel or Highest limit
    while (currentVoltage < voltageHighest && targetLevel > currentVoltage) {
      servoLift.setPosition(liftPowerScaledMoveUp);
      currentVoltage = liftVoltage.getVoltage();
      telemetry.addData("Lift Target Junction Level: ", targetLevel);
      telemetry.addData("Lift Current Voltage: ", currentVoltage);
      telemetry.update();
    }
    
    currentVoltage = stopLiftMovement();

    return currentVoltage;
    
  }  // end method moveToNextHigherJunctionLevelVoltage()

  
  @ExportToBlocks (
    heading = "Move To Next Lower Junction Level",
    color = 255,
    comment = "Move to next lower Junction level.",
    tooltip = "Move to next lower Junction level."
  )
  /**
   * Move lift down one Junction level 
   * Should check to make sure not reaching low limit
   */
  public static double moveToNextLowerJunctionLevelVoltage() {

    // Get our lastest voltage (aka lift position) while also stopping lift movement
    currentVoltage = stopLiftMovement();
    
    targetLevel = findNextLowerJunctionLevelVoltage();
    
    // Set information to be displayed
    telemetry.addData("Lift Request:", "Move To Next Lower Level" );
    telemetry.addData("Lift Target Junction Level: ", targetLevel);
    telemetry.addData("Lift Max Limit: ", currentVoltage);
    telemetry.addData("Lift Power To Move Down: ", liftPowerScaledMoveDown);
    telemetry.addData("Lift Max Limit: ", voltageLowest);

    telemetry.update();
  
    // if targetJunctionLevel is -1 or 0 then we don't move
    // will return the current voltage and get out of our method
    if (targetLevel <= 0.0)
      return currentVoltage;
      
    // Move down until able to reach our targetJunctionLevel or lowest limit
    while (currentVoltage > voltageLowest && targetLevel < currentVoltage) {
      servoLift.setPosition(liftPowerScaledMoveDown);
      currentVoltage = liftVoltage.getVoltage();
      telemetry.addData("Lift Target Level: ", targetLevel);
      telemetry.addData("Lift Current Voltage: ", currentVoltage);
    }
    
    currentVoltage = stopLiftMovement();

    return currentVoltage;
    
  }  // end method moveToNextLowerJunctionLevelVoltage()

  @ExportToBlocks (
    heading = "Move Up",
    color = 255,
    comment = "Move Lift up.",
    tooltip = "Move lift up."
  )
  /**
   * Move lift up if not at the highest limit
   */
  public static double moveUp() {
    
    // Make sure our event driven routines to lift to a target level will also stop
    liftMovingToTarget = false;
    
    // Set information to be displayed
    telemetry.addData("Lift Highest Limit: ", voltageHighest);
    
    // Get our lastest voltage (aka lift position)
    targetLevel = currentVoltage = liftVoltage.getVoltage();

    if (currentVoltage <= voltageHighest) {
      servoLift.setPosition(liftPowerScaledMoveUp);
      currentVoltage = liftVoltage.getVoltage();

    } else {
      currentVoltage = stopLiftMovement();
    }
    return currentVoltage;
  }  // end method moveUp()

  @ExportToBlocks (
    heading = "Move Down",
    color = 255,
    comment = "Move Lift down.",
    tooltip = "Move lift down."
  )
  /**
   * Move lift down if not at the lowest limit
   */
  public static double moveDown() {
    
    // Make sure our event driven routines to lift to a target level will also stop
    liftMovingToTarget = false;
    
    // Set information to be displayed
    telemetry.addData("Lift Power for Moving Down: ", liftPowerScaledMoveDown);
    telemetry.addData("Lift Lower Limit: ", voltageLowest);
    
    // Get our lastest voltage (aka lift position)
    targetLevel = currentVoltage = liftVoltage.getVoltage();

    if (currentVoltage >= voltageLowest) {
      servoLift.setPosition(liftPowerScaledMoveDown);
      currentVoltage = liftVoltage.getVoltage();
      telemetry.addData("Lift Action:", "Moving Down");
    } else {
      currentVoltage = stopLiftMovement();
    }
    return currentVoltage;
  }  // end method moveDown()

  @ExportToBlocks (
    heading = "Lift Info",
    color = 255,
    comment = "Sets telemetry to display for lift.",
    tooltip = "Sets telemetry to display for lift."
  )
  /**
   * Sets telemetry to display for lift
   */
  public static void setToDisplayLiftInfo() {
    telemetry.addData("Lift Power:", servoLift.getPosition());
    telemetry.addData("Lift Voltage:", currentVoltage);
  }  // end method setToDisplayLiftInfo()
  
}  // end class Lift