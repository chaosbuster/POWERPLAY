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
  static int parkingTileLocation = 0; 
  static int index = -1;

  static List<String> optionsInitialTilePosition = JavaUtil.createListWith();


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

    // Configuration settings if starting in A5
    optionsInitialTilePosition.add("A5");

    // Configuration settings if starting in F2
    optionsInitialTilePosition.add("F2");

    // Configuration settings if starting in F5
    optionsInitialTilePosition.add("F5");

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
  static public void initConfiguration(String ourInitialPosition) {
      
    // See if we were able to determine our Initial Position  
    if (ourInitialPosition.length() < 2) {
      // TODO: Position not found visually
      
    } else {
        
        // TODO:  Set robot configuration based on being in either a X or B position or RED or BLUE alliance

        // Find the Tile Position (eg. "A2") in our set of options which is an array.  
        // The index is the key to all the other configuration options.
        index = optionsInitialTilePosition.indexOf(ourInitialPosition) + 1;
        curTilePosition = (((String) JavaUtil.inListGet(optionsInitialTilePosition, JavaUtil.AtMode.FROM_START, (index - 1), false)));

        VisionAI.initCamerasBasedOnConfiguration(index);
      
        // Set information to display at the next telemetry update
      telemetry.addData("initConfiguration: Our Initial Position", ourInitialPosition);
      telemetry.addData("initConfiguration: index", index);
      telemetry.addData("initConfiguration: curTilePosition", curTilePosition);

    }
  }  // end method initConfiguration()


  @ExportToBlocks (
    heading = "SC: Parking Location",
    color = 32,
    comment = "Determines the Parking Tile Position in which we want to park",
    tooltip = "Assumes we are heading to the column that has the stacks of cones.",
    parameterLabels = {"Our Initial Position"}
  )
   /** Read the Parking Tile Position in which we want to park
    */
  static public int readParkingTilePosition(String ourInitialPosition) {
    // ASSUMPTIONS
    // Front camera is towards Signal
    // In Init so can't move cameras
    // *********************************
    index = VisionAI.identifyParkingLocationFromCustomSignal();
    
    // TODO: Parking Position versus Tile Row to Park per Initial Position
    
    // Set information to display at the next telemetry update    
    telemetry.addData("index", index);
    
    return index;
  } // end method readParkingTilePosition()


}
