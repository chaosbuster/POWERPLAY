package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.JavaUtil;


public class Lift extends BlocksOpModeCompanion {

  static AnalogInput liftVoltage;
  static Servo lift;

  // Java class variables for detecting & controlling lift range
  static double voltageLowest;
  static double voltageHighest;
  static double currentVoltage;
  
  // Java class variables for movement of the lift motor
  static double currentLiftPower;
  static double liftPowerScaledNoMovement;
  static double liftPowerScaledMoveUp;
  static double liftPowerScaledMoveDown;
  
  // Initialize our dropping lift levels
  // dropLevel: Range 0-1.00
    static double[] liftDropLevel = {0.55, 0.7, 0.85};  // {0.45, 0.84, 1.14}
    static double levelThreshold = 0.02;

  // Set our lift position to be mid-one
  // *** Will need to move the lift to initial lift level below on InitLift()
  static int indexLiftDropLevel = 1;
    
  // Variables to make sure we don't change the lift level too fast
  static double liftLevelElapsedTime = 0;
  static int liftLevelChangeAllowableTime = 500;
  static ElapsedTime liftLevelChangeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


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
    lift = hardwareMap.get(Servo.class, liftMotorName);
    liftVoltage = hardwareMap.get(AnalogInput.class, liftAnalogSensorName);
  
    // Readings for our analog sensor associated with lift
    // ***Values will vary if the cord and springs are stretched***
    voltageLowest = 0.45;  // 0.45  // static variables defined across java class
    voltageHighest = 1.19;  // 1.19  static variables defined across java class

    // Prototyped values to understand range of lift for scaled movement
    double liftPowerToMoveUp = 115;
    double liftPowerToMoveDown = 180;
    double liftPowerForNoMovement = 150;
    double liftPowerRange = 100;
    double liftPowerMin = 100;
    
    liftPowerScaledMoveUp = (liftPowerToMoveUp - liftPowerMin) / liftPowerRange;
    liftPowerScaledMoveDown = (liftPowerToMoveDown - liftPowerMin) / liftPowerRange;
    liftPowerScaledNoMovement = (liftPowerForNoMovement - liftPowerMin) / liftPowerRange;
    currentLiftPower = liftPowerScaledNoMovement;
    
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
    telemetry.addData("Lift Power for NO Movement:", liftPowerScaledNoMovement);
    
    // Get our lastest voltage (aka lift position)
    currentVoltage = liftVoltage.getVoltage();

    lift.setPosition(liftPowerScaledNoMovement);
    telemetry.addData("Lift Action:", "No Movement");
    return liftVoltage.getVoltage();
  }  // end method stopLiftMovement()

  @ExportToBlocks (
    heading = "Move Lift Up One Drop Level",
    color = 255,
    comment = "Move lift up one drop level.",
    tooltip = "Move lift up one drop level."
  )
  /**
   * Move lift up one drop level 
   * Should check to make sure not reaching high limit
   */
  public static double moveUpADropLevel() {
    double targetDropLevel;
    
    // Set information to be displayed
    telemetry.addData("Lift Power for Moving Up: ", liftPowerScaledMoveUp);
    telemetry.addData("Lift Highest Limit: ", voltageHighest);
    
    // Get our lastest voltage (aka lift position)
    currentVoltage = liftVoltage.getVoltage();
    
    // Set the level to the current lift level as default if we don't find a higher level
    targetDropLevel = currentVoltage;
    for (int i = 0; i < liftDropLevel.length; i++) {
      if (currentVoltage < liftDropLevel[i]) {
        // Found a higher drop level
        targetDropLevel = liftDropLevel[i];
        break;
      }
    }
    
    // If targetDropLevel is higher than our highest limit then don't go to it
    if (targetDropLevel >= voltageHighest)
      return currentVoltage;
    
    telemetry.addData("Lift Action:", "Moving Up");
    // Move up until able to reach our targetDropLevel or Highest limit
    while (currentVoltage < voltageHighest && targetDropLevel > currentVoltage) {
      lift.setPosition(liftPowerScaledMoveUp);
      currentVoltage = liftVoltage.getVoltage();
      telemetry.addData("Lift Target Drop Level: ", targetDropLevel);
      telemetry.addData("Lift Current Voltage: ", currentVoltage);
    }
    
    currentVoltage = stopLiftMovement();

    return currentVoltage;
  }  // end method moveUpADropLevel()

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
    
    // Set information to be displayed
    telemetry.addData("Lift Power for Moving Up: ", liftPowerScaledMoveUp);
    telemetry.addData("Lift Highest Limit: ", voltageHighest);
    
    // Get our lastest voltage (aka lift position)
    currentVoltage = liftVoltage.getVoltage();

    if (currentVoltage <= voltageHighest) {
      lift.setPosition(liftPowerScaledMoveUp);
      currentVoltage = liftVoltage.getVoltage();
      telemetry.addData("Lift Action:", "Moving Up");
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
    
    // Set information to be displayed
    telemetry.addData("Lift Power for Moving Down: ", liftPowerScaledMoveDown);
    telemetry.addData("Lift Lower Limit: ", voltageLowest);
    
    // Get our lastest voltage (aka lift position)
    currentVoltage = liftVoltage.getVoltage();

    if (currentVoltage >= voltageLowest) {
      lift.setPosition(liftPowerScaledMoveDown);
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
    telemetry.addData("Lift Power:", lift.getPosition());
    telemetry.addData("Lift Voltage:", currentVoltage);
  }  // end method setToDisplayLiftInfo()
  
}  // end class Lift