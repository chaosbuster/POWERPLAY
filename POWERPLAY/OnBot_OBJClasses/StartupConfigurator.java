package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionAI;


public class StartupConfigurator extends BlocksOpModeCompanion  {

  static String ourInitialPosition = "";
  static String curTilePosition = "";
  static int indexOfInitialTilePosition = -1;

  static List<String> optionsInitialTilePosition = JavaUtil.createListWith();
  static List<Integer> optionsRelativePositionForPL1 = JavaUtil.createListWith();
  static List<Integer> optionsRelativePositionForPL2 = JavaUtil.createListWith();
  static List<Integer> optionsRelativePositionForPL3 = JavaUtil.createListWith();

  @ExportToBlocks (
    heading = "Startup Configurator",
    color = 32,
    comment = "Sets up the robot's configuration based on what cameras see during Init.",
    tooltip = "Assumes the front camera is pointing forward and the back camera is pointing towards the closest wall Trackable."
  )
   /** Sets up the options of a robot's configuration
    */
  static public void setConfigurationOptions() {
    
    // TODO:  Set options for robot being in either setup X or B
    //optionsInitialTilePosition = JavaUtil.createListWith();
    
    // Configuration settings if starting in A2
    optionsInitialTilePosition.add("A2");
    optionsRelativePositionForPL1.add(2);
    optionsRelativePositionForPL2.add(1);
    optionsRelativePositionForPL3.add(0);

    // Configuration settings if starting in A5
    optionsInitialTilePosition.add("A5");
    optionsRelativePositionForPL1.add(0);
    optionsRelativePositionForPL2.add(1);
    optionsRelativePositionForPL3.add(2);
    
    // Configuration settings if starting in F2
    optionsInitialTilePosition.add("F2");
    optionsRelativePositionForPL1.add(0);
    optionsRelativePositionForPL2.add(1);
    optionsRelativePositionForPL3.add(2);

    // Configuration settings if starting in F5
    optionsInitialTilePosition.add("F5");
    optionsRelativePositionForPL1.add(2);
    optionsRelativePositionForPL2.add(1);
    optionsRelativePositionForPL3.add(0);

    // Set information to display at next telemetry update
    telemetry.addData("Our Tile Position", ourInitialPosition);

  }  // end method setConfigurationOptions()
  
  @ExportToBlocks (
    heading = "Startup Configurator",
    color = 32,
    comment = "Sets the robot configuration based on what the cameras saw during Init.",
    tooltip = "Assumes cameras were able to see a Trackable and a Signal during Init.",
    parameterLabels = {"Our Initial Position"}
  )
   /** Sets the actual robot configuration based on what cameras have seen during Init.
    */
  static public void SetOurInitialConfiguration(String ourInitialPosition) {
      
    // See if we were able to determine our Initial Position  
    if (ourInitialPosition.length() < 2) {
      // TODO: Position not found visually
      
    } else {
        
        // TODO:  Set robot configuration based on being in either a X or B position or RED or BLUE alliance

        // Find the Tile Position (eg. "A2") in our set of options which is an array.  
        // The index is the key to all the other configuration options.
        // Starting index is 0.
        indexOfInitialTilePosition= optionsInitialTilePosition.indexOf(ourInitialPosition);
        curTilePosition = (((String) JavaUtil.inListGet(optionsInitialTilePosition, JavaUtil.AtMode.FROM_START, (indexOfInitialTilePosition), false)));

        VisionAI.initCamerasBasedOnConfiguration(indexOfInitialTilePosition);

        // Set information to display at the next telemetry update
        telemetry.addData("initConfiguration: Our Initial Position", ourInitialPosition);
        telemetry.addData("initConfiguration: index", indexOfInitialTilePosition);
        telemetry.addData("initConfiguration: curTilePosition", curTilePosition);

    }
  }  // end method initConfiguration()


  @ExportToBlocks (
    heading = "Startup Configurator",
    color = 32,
    comment = "Determines the Relative Parking Tile Position in which we want to park",
    tooltip = "Relative position from tile [0] with cone stack. -1 if signal not found."
  )
   /** Read the Parking Tile Position in which we want to park
    */
  static public int readRelativeParkingLocation() {
    // ASSUMPTIONS
    // Front camera is towards Signal
    // In Init so can't move cameras
    // *********************************
    int signaledParkingLocation = -1;
    int relativePosition = -1;
    
    // Let's "see" what our signal says
    signaledParkingLocation = VisionAI.identifyParkingLocation();
    
    if (signaledParkingLocation == 1) {
      // Our signal is to park in Parking Location 1
      relativePosition = (((int) JavaUtil.inListGet(optionsRelativePositionForPL1, JavaUtil.AtMode.FROM_START, (indexOfInitialTilePosition), false)));
    } else if (signaledParkingLocation == 2) {
      // Our signal is to park in Parking Location 2
      relativePosition = (((int) JavaUtil.inListGet(optionsRelativePositionForPL2, JavaUtil.AtMode.FROM_START, (indexOfInitialTilePosition), false)));
    } else if (signaledParkingLocation == 3) {
      // Our signal is to park in Parking Location 2
      relativePosition = (((int) JavaUtil.inListGet(optionsRelativePositionForPL3, JavaUtil.AtMode.FROM_START, (indexOfInitialTilePosition), false)));
    } else {
      relativePosition = -1;
    }
      
    // Set information to display at the next telemetry update    
    telemetry.addData("Parking Location Signal ", signaledParkingLocation);
    telemetry.addData("Relative Parking Location from Cone Stack: ", relativePosition);
    
    return relativePosition;
  } // end method readRelativeParkingTilePosition()


}
