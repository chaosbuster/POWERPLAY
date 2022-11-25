package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;


public class Grabber extends BlocksOpModeCompanion {

  static public Servo servoGrabber;

  static double scaledPositionGrab;       // Calculated in the init
  static double scaledPositionRelease;    // Calculated in the init

  @ExportToBlocks (
    heading = "Initialize Grabber",
    color = 255,
    comment = "Initialize variables for our grabber.",
    tooltip = "Initialize variables for our grabber.",
    parameterLabels = {"Grabber Servo Name"}
  )
   /** Initialize variables for our grabber:
    *    > Hardware map handle for motors and sensors
    */
   public static void initGrabber(String GrabberServoName) {
      
      // Let's get a hardware handle on the servo for the grabber
      // The string provided should be the same as the name in the active hardware configuration 
      servoGrabber = hardwareMap.get(Servo.class, GrabberServoName);
      
      // Prototyped values of servo positions for associated grabbing actions
      double positionGrabberMinScale = 100;
      double positionGrabberMaxScale = 200;
      double positionGrabberMaxGrip = 150;
      double positionGrabberGoodGrip = 154;
      double positionGrabberTouchSides = 185;
      double positionGrabberClearSides = 182;
      double positionGrabberRelease = 170;
    
      // The two key servo positions that will be used 
      scaledPositionGrab = (positionGrabberGoodGrip - positionGrabberMinScale) / (positionGrabberMaxScale - positionGrabberMinScale);
      scaledPositionRelease = (positionGrabberRelease - positionGrabberMinScale) / (positionGrabberMaxScale - positionGrabberMinScale);
      
      // Set grabber positions to be displayed 
      telemetry.addData("Scaled Grab Position", scaledPositionGrab);
      telemetry.addData("Scaled Release Position", scaledPositionRelease);
  }

  @ExportToBlocks (
    heading = "Is Grabber Closed",
    color = 255,
    comment = "Returns whether the grabber is closed.",
    tooltip = "Returns a TRUE if grabber is closed."
  )
  /**
   * Determines whether the grabber is closed and returns:
   *    TRUE Grabber IS closed
   *    FALSE Grabber is NOT closed
   */
  public static boolean isGrabberClosed() {
    double servoPosition = servoGrabber.getPosition();
    
    // Check to see if grabber servo position is close enough to the close set position
    if (Math.abs(servoPosition - scaledPositionGrab) < 0.05) 
      return true;
    else
      return false;
    
  }  // end method isGrabberClosed()

  @ExportToBlocks (
    heading = "Close Grabber",
    color = 255,
    comment = "Action to close grabber and grab.",
    tooltip = "Action to close grabber and grab."
  )
  /**
   * Closing servo/grabber enough to hold the item
   */
  public static void closeGrabber() {
    servoGrabber.setPosition(scaledPositionGrab);
  }  // end method closeGrabber()

  @ExportToBlocks (
    heading = "Open Grabber",
    color = 255,
    comment = "Action to open grabber and release.",
    tooltip = "Action to open grabber and release."
  )
  /**
   * Opening servo/grabber enough to release the item
   */
  public static void openGrabber() {
    servoGrabber.setPosition(scaledPositionRelease);
  }  // end method openGrabber()
  
}  // end class Grabber
