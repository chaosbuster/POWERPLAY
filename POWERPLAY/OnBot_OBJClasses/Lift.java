package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import com.qualcomm.robotcore.hardware.LED;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;


public class Lift extends BlocksOpModeCompanion {

  static AnalogInput liftVoltage;
  static Servo servoLift;
  
  private static DistanceSensor sensor2MDistance;
  static int distanceMinToBlockLower = 120;
  static double curDistanceObjectToBackOfGrabber = 0;

  // Java class variables for detecting & controlling lift range
  static double voltageLowest = 0.43;  // 0.45  // static variables defined across java class
  static double voltageHighest = 1.31;  // 1.19  static variables defined across java class
  
  // Initialize our junction lift levels
  // liftDropLevel: Range voltageLowest - voltageHighest
  static double[] liftJunctionLevel = {0.46, 0.92, 1.24};  // {0.45, 0.84, 1.14}

  // Initialize our pickup lift levels
  // Range voltageLowest - voltageHighest
  static double[] liftPickupLevel = {0.45, 0.50, 0.56};  // {0.43, 0.47, 0.52, 0.55, 0.58}

  // Determines whether the lift is in an event driven mode of moving to a target
  static boolean liftMovingToTarget = false;
  static double targetLevel = voltageLowest;  
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

  // Run states for our lift
  // NOTE:  A target level must be set for the lift to raise or lower
  static private enum State {
      START,
      LOWER_LIFT, 
      STOP_LIFT,
      RAISE_LIFT,       
      END
  }
  
  static State liftState;
  
//====================================================================================  
//======================== CLASS METHODS =============================================
//====================================================================================

  @ExportToBlocks (
    heading = "Run Lift Based on State",
    color = 255,
    comment = "Runs lift based on the state of lift.",
    tooltip = "States are changed by using setting a target level."
  )
  /**
   * Runs an interation of the lift based on it's current state
   * Running the lift in a STATE mode allows for other actions to happen on the robot
   */
  public static void runLiftIteration() {
    
    // Get our lastest voltage (aka lift position)
    currentVoltage = liftVoltage.getVoltage();

    // Grab the latest distance reading at the bottom of the lift to see if anything there
    curDistanceObjectToBackOfGrabber = sensor2MDistance.getDistance(DistanceUnit.MM);
    telemetry.addData("Distance Read", curDistanceObjectToBackOfGrabber);
 
    switch (liftState) {
      
      case START: {
        break;
      }

      case LOWER_LIFT: {
        // Move down until able to reach our targetLevel or lowest limit
        
        // DO NOT LOWER LIFT IF SOMETHING IN THE GRABBER'S PATH
        if (curDistanceObjectToBackOfGrabber < distanceMinToBlockLower) {
          stopLiftMovement();               
          liftState = State.STOP_LIFT;
          
        } if (currentVoltage > voltageLowest && targetLevel < currentVoltage) {

          // Initiate moving down to target
          servoLift.setPosition(liftPowerScaledMoveDown);
        
          telemetry.addData("Lift Movement: ", "DOWN");
          telemetry.addData("Lift Target Level: ", targetLevel);
          telemetry.addData("Lift Current Voltage: ", currentVoltage);

        } else {
            
          stopLiftMovement();               
          liftState = State.STOP_LIFT;
        }

        break;
      }

      case STOP_LIFT: {
        
        stopLiftMovement();    
                
        telemetry.addData("Lift Movement: ", "STOP");
        telemetry.addData("Lift Last Target Level: ", targetLevel);
        telemetry.addData("Lift Current Voltage: ", currentVoltage);

        break;
      }

      case RAISE_LIFT: {
        
        if (currentVoltage < voltageHighest && targetLevel > currentVoltage) {
          
          // Initiate moving up to target
          servoLift.setPosition(liftPowerScaledMoveUp);
        
          telemetry.addData("Lift Movement: ", "UP");
          telemetry.addData("Lift Target Level: ", targetLevel);
          telemetry.addData("Lift Current Voltage: ", currentVoltage);
 
        } else {
          stopLiftMovement();               
          liftState = State.STOP_LIFT;
        }

        break;
      }

      case END: {
        break;
      }
    }

  }
  
  
  @ExportToBlocks (
    heading = "Initialize Lift",
    color = 255,
    comment = "Initialization of lift and variables.",
    tooltip = "Initialization of lift and variables.",
    parameterLabels = {"Lift Motor Name", "Lift Analog Sensor Name", "Sensor At Lift Bottom Name"}
  )
  /**
   * Initialization of lift and variables
   */
  public static void initLift(String liftMotorName, String liftAnalogSensorName, String sensor2MDistanceName) {
    
    liftState = State.START;

    // initialize our handles on hardware parts of our lift
    // String names provided to this function should be the same as those in active hardware configuration
    servoLift = hardwareMap.get(Servo.class, liftMotorName);
    liftVoltage = hardwareMap.get(AnalogInput.class, liftAnalogSensorName);
 
    telemetry.addData("Lowest voltage", Double.parseDouble(JavaUtil.formatNumber(voltageLowest, 2)));
    telemetry.addData("Highest voltage", Double.parseDouble(JavaUtil.formatNumber(voltageHighest, 2)));
    
    // Let's get a hardware handle on our distance sensor at the bottom of our lift
    sensor2MDistance = hardwareMap.get(DistanceSensor.class, sensor2MDistanceName);
    
    // Grab the current distance reading 
    curDistanceObjectToBackOfGrabber = sensor2MDistance.getDistance(DistanceUnit.MM);
    telemetry.addData("Distance Read", curDistanceObjectToBackOfGrabber);
 
    
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
  public static void stopLiftMovement() {

    servoLift.setPosition(liftPowerScaledNoMovement);
    
  }  // end method stopLiftMovement()
 
  @ExportToBlocks (
    heading = "Move To Level Specified",
    color = 255,
    comment = "Move to level[0,1,2] specified by button selected[A, B/X, Y] and whether grabber is closed.",
    tooltip = "Move to a level. Send -1 to check status if ready to stop.",
    parameterLabels = {"Level Request", "Is Grabber Closed"}
  )
  /**
   * Initiate a move of the lift to a level requested using our STATEs.
   * By using STATEs the action of the lift moving up/down does not block  
   * the drivetrain and other actions requested by drivers.
   * Should check to make sure not reaching over the high or below low limit
   */
  public static void moveToLevelSpecified(int levelRequested, boolean grabberClosed) {
    
    // invalid index to an array
    if (levelRequested < 0) {
      stopLiftMovement();
      return;
    }

    // Determine which level [Junction or Pickup] is being requesting
    // Based off of whether the Grabber is closed
    if (Grabber.isGrabberClosed()) {
      // Grabber closed so assuming the level requested is a Junction level
      
      // if an invalid index to the Junction levels, then just return and don't do anything
      if (levelRequested > liftJunctionLevel.length - 1) {
        stopLiftMovement();
        return;
      }
      
      // Set our target voltage level to the level requested  
      targetLevel = liftJunctionLevel[levelRequested];
      
    } else {
      // Grabber open so assuming the level requested is a pickup level
      
      // if an invalid index to the pickup levels, then just return and don't do anything
      if (levelRequested > liftPickupLevel.length - 1) {
        stopLiftMovement();
        return;
      }
      
      // Set our target voltage level to the level requested  
      targetLevel = liftPickupLevel[levelRequested];
      
    }
    
    // if target level is past our maximum limit then return and don't do anything
    if (targetLevel > voltageHighest ) {
      targetLevel = voltageHighest;
    // if target level is past our low limit then return and don't do anything
    } else if (targetLevel < voltageLowest) {
      targetLevel = voltageLowest;
    }

    // Get our lastest voltage (aka lift position)
    currentVoltage = liftVoltage.getVoltage();

    // Set information to be displayed
    telemetry.addData("Lift TARGET LEVEL: ", targetLevel);
    telemetry.addData("Lift CURRENT VOLTAGE: ", currentVoltage);
    
    // If target level is higher than our current level then initiate moving up
    if (isTargetLevelHigher(targetLevel)) {
      // Move up until able to reach our target drop level or high limit
      if (currentVoltage < voltageHighest && targetLevel > currentVoltage) {
          
        // Set our lift state to move up
        liftState = State.RAISE_LIFT;

        // Initiate moving up to target
        //servoLift.setPosition(liftPowerScaledMoveUp);
        
        telemetry.addData("Lift Movement: ", "UP");

      } else {
        stopLiftMovement();
      }
     
    
    // If target level is lower than our current level then initiate moving down
    } else {
      // Move down until able to reach our targetLevel or lowest limit
      if (currentVoltage > voltageLowest && targetLevel < currentVoltage) {

        // Set our lift state to move up
        liftState = State.LOWER_LIFT;  
        
        telemetry.addData("Lift Movement: ", "DOWN");

        
      } else {
        stopLiftMovement();

      }
      
    }
    
  }  // end method moveToNextHigherDropLevelVoltage()

  /**
   * Checks to see if we have already reached our target lift level
   * before setting a lift state to RAISE or LOWER the lift.
   * Should also check to make sure not reaching high or low limit
   */
  private static boolean targetLevelReached() {
      
    // Get our lastest voltage (aka lift position)
    currentVoltage = liftVoltage.getVoltage();
    
    // See if we are within acceptable range of our target level
    if (Math.abs(currentVoltage - targetLevel) < targetThreshold) {
      stopLiftMovement();
      return true;
    }

    // Stop if we are close to our limits    
    if ((currentVoltage <= voltageLowest) || (currentVoltage >= voltageHighest)) {
      stopLiftMovement();
      return true;
    }

    return false;
    
  }  // end method targetLevelReached()
  
    /**
   *  Determines where the target level is based on the current voltage
   */
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
    telemetry.addData("Lift Current Servo Position:", servoLift.getPosition());
    telemetry.addData("Lift Voltage:", currentVoltage);

  }  // end method setToDisplayLiftInfo()
  
//====================================================================================  
//============== METHODS BELOW NO LONGER USED.  THEY BLOCK OTHER ACTIONS ============
//====================================================================================
  // Internal function to find the next higher junction level ONLY
  // Status: NOT USED
  // A more optimal solution was coded with lift STATEs
  private static double findNextHigherJunctionLevelVoltage() {
    /*   Returns the following:
     *        -1  Higher level is greater than lift maximum height limit
     *         0  No higher Junction level compared to current voltage
     *   +double  Voltage of Junction level higher than current voltage
     */

    // Get our lastest voltage (aka lift position)
    currentVoltage = liftVoltage.getVoltage();
    
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

  // Internal function to find the next lower junction level ONLY
  // Status: NOT USED
  // A more optimal solution was coded with lift STATEs
  private static double findNextLowerJunctionLevelVoltage() {
    /*   Returns the following:
     *        -1  Lower level is lower than lift lowest height limit
     *         0  No lower Junction level compared to current voltage
     *   +double  Voltage of junction level lower than current voltage
     *   ASSUMPTION: Voltage is always positive.
     */

    // Get our lastest voltage (aka lift position)
    currentVoltage = liftVoltage.getVoltage();
    
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
  
  /**
   * Move lift up one Junction level 
   * Should check to make sure not reaching high limit
   * WARNING: This function will block all actions until done.  
   * Status: NOT USED
   * A more optimal solution was coded with lift STATEs
   */
  public static void moveToNextHigherJunctionLevelVoltage() {
    
    // Get our lastest voltage (aka lift position)
    currentVoltage = liftVoltage.getVoltage();
    
    targetLevel = findNextHigherJunctionLevelVoltage();
    
    // Set information to be displayed
    telemetry.addData("Lift Request:", "Move To Next Higher Level" );
    telemetry.addData("Lift Target Junction Level: ", targetLevel);
    telemetry.addData("Lift Max Limit: ", currentVoltage);

    telemetry.update();
  
    // if targetJunctionLevel is -1 or 0 then we don't move up
    // will return the current voltage and get out of our method
    if (targetLevel <= 0.0)
      return;
    

    // Move up until able to reach our targetDropLevel or Highest limit
    while (currentVoltage < voltageHighest && targetLevel > currentVoltage) {
      servoLift.setPosition(liftPowerScaledMoveUp);
      currentVoltage = liftVoltage.getVoltage();
      telemetry.addData("Lift Target Junction Level: ", targetLevel);
      telemetry.addData("Lift Current Voltage: ", currentVoltage);
      telemetry.update();
    }
        
    stopLiftMovement();
    
    return;
    
  }  // end method moveToNextHigherJunctionLevelVoltage()

  
  /**
   * Move lift down one Junction level 
   * Should check to make sure not reaching low limit
   * WARNING: This function will block all actions until done.  
   * Status: NOT USED
   * A more optimal solution was coded with lift STATEs
   */
  public static void moveToNextLowerJunctionLevelVoltage() {

    // Get our lastest voltage (aka lift position)
    currentVoltage = liftVoltage.getVoltage();
    
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
      return;
      
    // Move down until able to reach our targetJunctionLevel or lowest limit
    while (currentVoltage > voltageLowest && targetLevel < currentVoltage) {
      servoLift.setPosition(liftPowerScaledMoveDown);
      currentVoltage = liftVoltage.getVoltage();
      telemetry.addData("Lift Target Level: ", targetLevel);
      telemetry.addData("Lift Current Voltage: ", currentVoltage);
    }
    
    stopLiftMovement();

    return;
    
  }  // end method moveToNextLowerJunctionLevelVoltage()


}  // end class Lift
