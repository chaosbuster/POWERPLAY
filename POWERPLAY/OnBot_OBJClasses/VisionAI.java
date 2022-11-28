package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;

public class VisionAI extends BlocksOpModeCompanion  {

  static String curTilePosition;  // String with format of "Column" + "Row"
  
  static public VuforiaCurrentGame vuforiaPOWERPLAY = new VuforiaCurrentGame();
  static public Tfod tfod = new Tfod();

   // Vuforia names for all of our Trackable images on the field
  static final String constantTrackableB1 = "Blue Audience Wall";
  static final String constantTrackableB6 = "Blue Rear Wall";
  static final String constantTrackableE1 = "Red Audience Wall";
  static final String constantTrackableE6 = "Red Rear Wall";
  
 // Handles for each of our webcams
  static final String constantWebcam1 = "Webcam 1";
  static final String constantWebcam2 = "Webcam 2";

  static String activeCamera;
  static String camViewFront;
  static String camViewSide;
  
  static final int constantTimeBetweenCamerasMSec = 5000;
  static ElapsedTime timerSwitchCameras = new ElapsedTime();
  
  static int ParkingLocationOfLabel;
 
  // Default POWERPLAY Signal Tensorflow Model
  // 1 Gear, 2 Bear,3 Scratch
  static final String tfDefaultLabelParkingLocation1 = "Gear";
  static final String tfDefaultLabelParkingLocation2 = "Bear";
  static final String tfDefaultLabelParkingLocation3 = "Scratch";
  
  static String[] labelsCustomTFModel = {"Gear", "Bear", "Scratch"};
  
    
  // Custom Signal Tensorflow Model
  // 1 Gear, 2 Bear, 3 Scratch
  static final String tfCustomLabelParkingLocation1 = "Gear";
  static final String tfCustomLabelParkingLocation2 = "Bear";
  static final String tfCustomLabelParkingLocation3 = "Scratch";

  // Initialize our Lists for configuration settings
  static List<String> optionsInitialTilePosition; 
  static List<String> optionsTrackable;
  static List<String> optionsCamViewFront;
  static List<String> optionsCamViewSide;
  
  // Initialize our Lists for configuration settings
  //static List<String> optionsInitialTilePosition = {"A2","A5", "F2","F5" }; 
  //static List<String> optionsTrackable = {constantTrackableB1, constantTrackableB6, constantTrackableE1, constantTrackableE6};
  //static List<String> optionsCamViewFront = {constantWebcam1, constantWebcam2, constantWebcam2, constantWebcam1};
  //static List<String> optionsCamViewSide = {constantWebcam2, constantWebcam1, constantWebcam1, constantWebcam2};


  @ExportToBlocks (
    heading = "Vision",
    color = 32,
    comment = "Helps define the configuration of the robot based on what camera sees during Init",
    tooltip = "Options include InitialTilePosition, Adjacent Trackable and which camera is in Front or Side camera."
  )
   /** Helps define the configuration of the robot based on what camera sees during Init
    */
  static public void initVisionAI() {
    // Initialize Vision
    
    // Camera handles to use
    camViewFront = constantWebcam2;
    camViewSide = constantWebcam1;
    activeCamera = constantWebcam1;
    
    // Initialize our lists
    optionsInitialTilePosition = JavaUtil.createListWith();
    optionsTrackable = JavaUtil.createListWith();
    optionsCamViewFront = JavaUtil.createListWith();
    optionsCamViewSide = JavaUtil.createListWith();
    
    // Configuration settings if starting in A2
    optionsInitialTilePosition.add("A2");
    optionsTrackable.add(constantTrackableB1);
    optionsCamViewFront.add(constantWebcam1);
    optionsCamViewSide.add(constantWebcam2);
    
    // Configuration settings if starting in A5
    optionsInitialTilePosition.add("A5");
    optionsTrackable.add(constantTrackableB6);
    optionsCamViewFront.add(constantWebcam2);
    optionsCamViewSide.add(constantWebcam1);
    
    // Configuration settings if starting in F2
    optionsInitialTilePosition.add("F2");
    optionsTrackable.add(constantTrackableE1);
    optionsCamViewFront.add(constantWebcam2);
    optionsCamViewSide.add(constantWebcam1);
    
    // Configuration settings if starting in F5
    optionsInitialTilePosition.add("F5");
    optionsTrackable.add(constantTrackableE6);
    optionsCamViewFront.add(constantWebcam1);
    optionsCamViewSide.add(constantWebcam2);
    
    //Set information to display at next telemetry update
    telemetry.addData("camViewFront", camViewFront);
    telemetry.addData("camViewSide", camViewSide);
    telemetry.addData("Active Camera", activeCamera);
    telemetry.addData("Current Tile Position", curTilePosition);
 
  } // end method initVisionAI()
 
  @ExportToBlocks (
    heading = "Vision AI",
    color = 32,
    comment = "Initialize Vuforia with switchable web cameras",
    tooltip = "Initialize Vuforia with switchable web cameras"
  )
   /** Initialize Vuforia with switchable web cameras
    */
  static public void initVuforia() {
      
    // Initialize Vuforia using SwitchableCamera
    vuforiaPOWERPLAY.initialize(
        "", // vuforiaLicenseKey
        VuforiaBase.getSwitchableCamera(hardwareMap), // cameraName
        "", // webcamCalibrationFilename
        false, // useExtendedTracking
        true, // enableCameraMonitoring
        VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
        0, // dx
        0, // dy
        0, // dz
        AxesOrder.XZY, // axesOrder
        90, // firstAngle
        90, // secondAngle
        0, // thirdAngle
        true); // useCompetitionFieldTargetLocations
        
  }  // end method initVuforia()

  @ExportToBlocks (
    heading = "Vision AI",
    color = 32,
    comment = "Activate Vuforia.",
    tooltip = "Assumes Vuforia has been initialized."
  )
   /** Activate Vuforia
    */
  static public void activateVuforia(){
    vuforiaPOWERPLAY.activate();
  }  // end method activateVuforia() 

  @ExportToBlocks (
    heading = "Vision AI",
    color = 32,
    comment = "Deactivate Vuforia.",
    tooltip = "Assumes Vuforia is active."
  )
   /** Deactivate Vuforia
    */
  static public void deactivateVuforia(){
    vuforiaPOWERPLAY.deactivate();
  }  // end method deactivateVuforia() 

  @ExportToBlocks (
    heading = "Vision AI",
    color = 32,
    comment = "Close Vuforia.",
    tooltip = "Assumes Vuforia has been deactivated."
  )
   /** Close Vuforia
    */
  static public void closeVuforia(){
    vuforiaPOWERPLAY.close();
  }  // end method closeVuforia() 
  

  @ExportToBlocks (
    heading = "Vision AI",
    color = 32,
    comment = "Currently using for custom Parking Signal.",
    tooltip = "Currently using for custom Parking Signal."
  )
   /** Initialize tensorflow based on the type of model we have trained.
    */
  static public void initTensorflow() {

    // Set isModelTensorFlow2 to true if you used a TensorFlow 2 tool, such as ftc-ml, to create the model. 
    // Set isModelQuantized to true if the model is quantized. Models created with ftc-ml are quantized. 
    // Set inputSize to the image size corresponding to the model. 
    //    If your model is based on SSD MobileNet v2 320x320, the image size is 300 (srsly!). 
    //    If your model is based on SSD MobileNet V2 FPNLite 320x320, the image size is 320.
    //    If your model is based on SSD MobileNet V1 FPN 640x640 or SSD MobileNet V2 FPNLite 640x640, the image size is 640.
    
    tfod.useModelFromFile("GBS_253_ssd_v2_fpnlite_320x320_metadata.tflite", labelsCustomTFModel, true, true, 320);
    
    // Set min confidence threshold to 0.7
    tfod.initialize(vuforiaPOWERPLAY, (float) 0.7, true, true);
    
    // Enable following block to zoom in on target.
    tfod.setZoom(1, 16 / 9);
    
  }  // end method initTensorflow()
  
  
  @ExportToBlocks (
    heading = "Vision AI",
    color = 32,
    comment = "Activate Tensorflow.",
    tooltip = "Assumes Tensorflow has been initialized."
  )
   /** Activate Tensorflow
    */
  static public void activateTensorflow(){
    tfod.activate();
  }  // end method activateTensorflow() 

  @ExportToBlocks (
    heading = "Vision AI",
    color = 32,
    comment = "Deactivate Tensorflow.",
    tooltip = "Assumes Tensorflow is active."
  )
   /** Deactivate Tensorflow
    */
  static public void deactivateTensorflow(){
    tfod.deactivate();
  }  // end method deactivateTensorflow() 

  @ExportToBlocks (
    heading = "Vision AI",
    color = 32,
    comment = "Close Tensorflow.",
    tooltip = "Assumes Tensorflow has been deactivated."
  )
   /** Close Tensorflow
    */
  static public void closeTensorflow(){
    tfod.close();
  }  // end method closeTensorflow() 
    


  @ExportToBlocks (
    heading = "Vision AI",
    color = 32,
    comment = "Sets an initial value for the active.",
    tooltip = "Vuforia should have already been initialized."
  )
   /** Sets an initial value for the active camera.  
    *  Includes setting the active camera for Vuforia.
    */
  static public void initCameraSwitching() {
      
    // active camera to side view camera
    activeCamera = constantWebcam1;
    
    // After Vuforia is initialized, set the active camera
    vuforiaPOWERPLAY.setActiveCamera(hardwareMap.get(WebcamName.class, constantWebcam1));
    
  }  // end method initCameraSwitching()

  @ExportToBlocks (
    heading = "Vision AI",
    color = 32,
    comment = "Will set our cameras based on the configuration index provided.",
    tooltip = "Index provided is base 1.",
    parameterLabels = {"Configuration Index with Base 1"}
  )
   /** About to leave our Init stage and go into action
    *  We need to set our cameras based on an assumed configuration when we started our Init
    *  Assuming robot was in an X or B configuration and the grabber was towards the closest wall Trackable.
    */
  static public void initCamerasBasedOnConfiguration(int index) {
    // NEED TO CONFIRM options in the configurations
    
    // TODO:  NEED TO SET CAMERAS BASED ON CONFIGURATION DISCOVERED
    camViewFront = (((String) JavaUtil.inListGet(optionsCamViewFront, JavaUtil.AtMode.FROM_START, (index - 1), false)));
    camViewSide = (((String) JavaUtil.inListGet(optionsCamViewSide, JavaUtil.AtMode.FROM_START, (index - 1), false)));
    
    // After Vuforia is initialized, set to our side view to track trackables
    vuforiaPOWERPLAY.setActiveCamera(hardwareMap.get(WebcamName.class, camViewSide));
    
     // Set display information for the next telemetry update
    telemetry.addData("initConfiguration: camViewFront", camViewFront);
    telemetry.addData("initConfiguration: camViewSide", camViewSide);
 }  // end method initCamerasBasedOnConfiguration()

  @ExportToBlocks (
    heading = "Vision AI",
    color = 32,
    comment = "Switches to the Front camera.",
    tooltip = "The specific camera view depends on the robot orientation."
  )
    /** Switches to the Front camera.  
    */
  static public void switchToFrontView() {
      
    // Is the active camera already the side camera?
    if (!activeCamera.equals(camViewFront)) {
    
      // If not the side camera then let's switch, but if in the Init of the OpMode our configuration hasn't been set yet
      if (camViewFront.equals(constantWebcam1)) {
        // Check and Set by the actual Webcam Name handles
        vuforiaPOWERPLAY.setActiveCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        activeCamera = constantWebcam1;
      } else if (camViewFront.equals(constantWebcam2)) {
        vuforiaPOWERPLAY.setActiveCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
        activeCamera = constantWebcam2;
      }
    }
    // Set display information for the next telemetry update
    telemetry.addData("activeCamera", activeCamera);
    telemetry.addData("camViewFront", camViewFront);
  }  // end method switchToFrontView()

  @ExportToBlocks (
    heading = "Vision AI",
    color = 32,
    comment = "Switches to the Side (back) camera.",
    tooltip = "The specific camera view depends on the robot orientation."
  )
   /** Switches to the Side (back) camera.  
    */
  static public void switchToSideView() {
      
    // Is the active camera already the side camera?
    if (!activeCamera.equals(camViewSide)) {
    
      // If not the side camera then let's switch, but if in the Init of the OpMode our configuration hasn't been set yet
      if (camViewSide.equals(constantWebcam1)) {
        // Check and Set by the actual Webcam Name handles
        vuforiaPOWERPLAY.setActiveCamera(hardwareMap.get(WebcamName.class, constantWebcam1));
        activeCamera = constantWebcam1;
      } else if (camViewSide.equals(constantWebcam2)) {
        vuforiaPOWERPLAY.setActiveCamera(hardwareMap.get(WebcamName.class, constantWebcam2));
        activeCamera = constantWebcam2;
      }
    }
    
    // Set display information for the next telemetry update
    telemetry.addData("activeCamera", activeCamera);
    telemetry.addData("camViewSide", camViewSide);
  }  // end method switchToSideView()

  @ExportToBlocks (
    heading = "Vision AI",
    color = 32,
    comment = "Switches between a Front and Side camera.",
    tooltip = "The camera can only be switched every 5 seconds.  Assuming this is enough time for an image to be processed."
  )
   /** Switches between a Front and Side camera. 
    *  The camera can only be switched every 5 seconds.  Set via the constant constantTimeBetweenCamerasMSec in the class
    *  Assuming this is enough time for an image to be processed.
    */
  static public void switchCameraView() {
      
    // Check to see if we are trying to switch too often
    if (timerSwitchCameras.milliseconds() < constantTimeBetweenCamerasMSec) {
        // If requested switch too early than let's break out of the function
        telemetry.addData("SwitchCameraView", "TIMER SAYS TOO EARLY TO SWITCH CAMERAS");
        return;
    }
    
    // If our currently active camera is the Front camera...
    if (activeCamera.equals(camViewFront)) {
        
      // Switch to the side (back camera)
      switchToSideView();
      
    } else {
        
      // Switch to the front camera
      switchToFrontView();
    }
    
    // Resetting our timer for another cycle
    timerSwitchCameras.reset();
    
  }  // end method switchCameraView() 

  @ExportToBlocks (
    heading = "Vision AI",
    color = 32,
    comment = "Identifies first visible Trackable to determine our initial tile position.",
    tooltip = "We are in an Init routine and the robot is placed on 1 of the 4 starting positions with back camera pointed to closest Trackable."
  )
   /** Identifies first visible Trackable to determine our initial tile position
    *  Returns a Column Row String (eg "A2")
    *  ASSUMPTIONS:  
    *     <1> We are in an Init routine and the robot is placed on 1 of the 4 starting positions.
    *     <2> Robot is in a X or B Configuration.
    *     <3> Back camera was manually set to point towards the closest wall's Trackable
    *     <4> In Init so can't move cameras
    */
  static public String determineInitialTilePosition() {
    int index = -1;
    String closestTrackable;  // Trackable name, if found
    
    // Is a Trackable visible?  Returns the 
    closestTrackable = identifyVisibleTrackable();
    
    // Check to see if a Trackable was visible
    if (closestTrackable.equals("")) {
      // A Trackable was NOT visible
      // TODO: SET ROBOT TO NOT RUN AUTONOMOUS
      curTilePosition = "";
      
    } else {
      // A Trackable WAS visible
      index = optionsTrackable.indexOf(closestTrackable);  
      
      telemetry.addData("index", index);

      curTilePosition = (((String) JavaUtil.inListGet(optionsInitialTilePosition, JavaUtil.AtMode.FROM_START, (index), false)));
  
    }
    
    // Add telemetry display information for the next telemetry update
    telemetry.addData("Closest Trackable", closestTrackable);
    telemetry.addData("curTilePosition", curTilePosition);
    
    return curTilePosition;
    
  }  // end method determineInitialTilePosition()

  //@ExportToBlocks (
  //  heading = "Trackables",
  //  color = 32,
  //  comment = "Runs through each potential Trackable to see if it is visible",
  //  tooltip = "Returns the Trackable name that is found. Returns null String if none are found."
  //)
   /** Runs through each potential Trackable to see if it is visible
    *  Returns the Trackable name that is found.
    */
  static public String identifyVisibleTrackable() {
      
    String Trackable;
    
    // Goes through all four of the potential Trackables to see if it sees one
    // NOTE:  Since we are using a IF-ELSE-IF statement it will stop looking after it finds one.
    if (isTargetVisible(constantTrackableB1)) {
      Trackable = constantTrackableB1;
    } else if (isTargetVisible(constantTrackableB6)) {
      Trackable = constantTrackableB6;
    } else if (isTargetVisible(constantTrackableE1)) {
      Trackable = constantTrackableE1;
    } else if (isTargetVisible(constantTrackableE6)) {
      Trackable = constantTrackableE6;
    } else {
      Trackable = "";
    }
    return Trackable;
  }  // end method identifyVisibleTrackable()

  //@ExportToBlocks (
  //  heading = "Trackables",
  //  color = 32,
  //  comment = "Runs through the TrackingResults to see if the specified Trackable is visible" +
  //            "Returns true if the given Trackable by Name is found.",
  //  tooltip = "A trackable is not always visible based obstacles in the way of the cameras.",
  //  parameterLabels = {"Trackable Name"}
  //)
   /** Runs through the TrackingResults to see if the specified Trackable is visible
    *  Returns true if the given Trackable by Name is found.
    */
  static public boolean isTargetVisible(String trackableName) {
    boolean isVisible;
    VuforiaBase.TrackingResults vuforiaResults;

    // Get vuforia results for target
    vuforiaResults = vuforiaPOWERPLAY.track(trackableName);
    // Is this target visible?
    if (vuforiaResults.isVisible) {
      isVisible = true;
    } else {
      isVisible = false;
    }
    telemetry.addData("Detecting", vuforiaResults.name);
    telemetry.addData("Visible?", isVisible);
    return isVisible;
  }  // end method isTargetVisible()

  //@ExportToBlocks (
  //  heading = "Parking Signal",
  //  color = 32,
  //  comment = "Looks through all the object recognitions and determine which parking location it correlates to, if any." +
  //            "The parking location are for a default signal tensorflow model.",
  //  tooltip = "Returns [1,2,3] or 0 if did not recognize any signals in the line of sight."
  //)
   /** Runs through all the object recognitions
    *  Compares an object label from our known custom signal labels
    *  Returns the Parking Location if Signal found.  Otherwise, returns 0.
    */
  static public int identifyParkingLocationFromDefaultSignal() {
    String foundLabel;
    int i;
    Recognition recognition;
    List<Recognition> recognitions;
    int ParkingLocationOfLabel = 0;

    // Get a static List of recognitions from TFOD.
    recognitions = tfod.getRecognitions();
    
    // If static List is empty, inform the user. Otherwise, go
    // through static List and display info for each recognition.
    if (JavaUtil.listLength(recognitions) == 0) {
      telemetry.addData("TFOD", "No items detected.");
      i = 0;
    } else {
      i = 1;
      
      // Iterate through static List and call a function to
      // display info for each recognized object.
      for (Recognition recognition_item : recognitions) {
        recognition = recognition_item;
        foundLabel = recognition.getLabel();
        ParkingLocationOfLabel = labelIndexFromDefaultPOWERPLAY(foundLabel);

        // Display info on next telemetry update
        displaySignalInfo(i, recognition);
        
        // Increment index.
        i = i + 1;
      }
    }
    return ParkingLocationOfLabel;
  }  // end method identifyParkingLocationFromDefaultSignal()

  //@ExportToBlocks (
  //  heading = "Parking Signal",
  //  color = 32,
  //  comment = "Given a label string it will determine whether it correlates to a parking location." +
  //            "The parking location are for a default signal tensorflow model.",
  //  tooltip = "The parking location returned is [1,2,3]." +
  //            "If 0 is returned it did not find a correlated label to the string provided.",
  //  parameterLabels = {"Found Label"}
  //)
   /** Compares an object label from our known custom signal labels
    */
  static public int labelIndexFromDefaultPOWERPLAY(String foundLabel)  {
    
    int ParkingLocationOfLabel = 0;
    
    telemetry.addData("Label Found", foundLabel);
    if (foundLabel.equals(tfDefaultLabelParkingLocation1)) {
      ParkingLocationOfLabel = 1;
    } else if (foundLabel.equals(tfDefaultLabelParkingLocation2)) {
      ParkingLocationOfLabel = 2;
    } else if (foundLabel.equals(tfDefaultLabelParkingLocation3)) {
      ParkingLocationOfLabel = 3;
    }
    return ParkingLocationOfLabel;
  }  // end method labelIndexFromDefaultPOWERPLAY()


  //@ExportToBlocks (
  //  heading = "Parking Signal",
  //  color = 32,
  //  comment = "Looks through all the object recognitions and determine which parking location it correlates to, if any." +
  //            "The parking location are for a custom signal tensorflow model.",
  //  tooltip = "Returne [1,2,3] or 0 if did not recognize any signals in the line of sight."
  //)
   /** Runs through all the object recognitions
    *  Compares an object label from our known custom signal labels
    *  Returns the Parking Location if Signal found.  Otherwise, returns 0.
    */
  static public int identifyParkingLocationFromCustomSignal() {
    String foundLabel;
    int i;
    Recognition recognition;
    List<Recognition> recognitions;
    int ParkingLocationOfLabel = 0;

    // Get a static List of recognitions from TFOD.
    recognitions = tfod.getRecognitions();
    
    // If static List is empty, inform the user. Otherwise, go
    // through static List and display info for each recognition.
    if (JavaUtil.listLength(recognitions) == 0) {
      telemetry.addData("TFOD", "No items detected.");
      i = 0;
    } else {
      i = 1;
      
      // Iterate through static List and call a function to
      // display info for each recognized object.
      for (Recognition recognition_item : recognitions) {
        recognition = recognition_item;
        foundLabel = recognition.getLabel();
        ParkingLocationOfLabel = labelIndexFromCustomSignal(foundLabel);
        
        // Display info during the next telemetry update
        displaySignalInfo(i, recognition);
        
        // Increment index.
        i = i + 1;
      }
    }
    return ParkingLocationOfLabel;
  }  // end method identifyParkingLocationFromCustomSignal()

  //@ExportToBlocks (
  //  heading = "Parking Signal",
  //  color = 32,
  //  comment = "Given a label string it will determine whether it correlates to a parking location." +
  //            "The parking location are for a custom signal tensorflow model.",
  //  tooltip = "The parking location returned is [1,2,3]." +
  //            "If 0 is returned it did not find a correlated label to the string provided.",
  //  parameterLabels = {"Found Label"}
  //)
   /** Compares an object label from our known custom signal labels
    */
  static public int labelIndexFromCustomSignal(String foundLabel) {
    int ParkingLocationOfLabel = 0;
    
    telemetry.addData("Label Found", foundLabel);
    if (foundLabel.equals(tfCustomLabelParkingLocation1)) {
      ParkingLocationOfLabel = 1;
    } else if (foundLabel.equals(tfCustomLabelParkingLocation2)) {
      ParkingLocationOfLabel = 2;
    } else if (foundLabel.equals(tfCustomLabelParkingLocation3)) {
      ParkingLocationOfLabel = 3;
    }
    return ParkingLocationOfLabel;
  }  // end method labelIndexFromCustomSignal()

 
  @ExportToBlocks (
    heading = "Parking Signal",
    color = 32,
    comment = "Adds information to telemetry to be displayed.",
    tooltip = "A telemetry update will need to be called to display this information from the queue.",
    parameterLabels = {"Index of Object Recognized", "Recognition to Display"}
  )  // end method 
   /**  Gets the detail information about the object recognized in list of with index i
    *   Adds the detail information into telemetry to be displayed on driver station
    */
  static public void displaySignalInfo(int i, Recognition recognition) {

    // Make sure this section is visible and separateed on the driver station with an all CAPS header
    telemetry.addData("OBJECT DETECTED","Displaying Information");
    
    // Display label info.
    // Display the label and index number for the recognition.
    telemetry.addData("label " + i, recognition.getLabel());

    // Display upper corner info.
    // Display the location of the top left corner
    // of the detection boundary for the recognition
    telemetry.addData("Left, Top " + i, Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getTop(), 0)));
 
    // Display lower corner info.
    // Display the location of the bottom right corner
    // of the detection boundary for the recognition
    telemetry.addData("Right, Bottom " + i, Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getBottom(), 0)));
  }  // end method displaySignalInfo()

}
