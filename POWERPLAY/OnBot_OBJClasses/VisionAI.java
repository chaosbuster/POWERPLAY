package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;


import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class VisionAI extends BlocksOpModeCompanion  {
    
    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    //private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String TFOD_MODEL_FILE  = "GBS_253_ssd_v2_fpnlite_320x320_metadata.tflite";

    private static final String[] LABELS = {
            "1 Gear",
            "2 Bear",
            "3 Scratch"
    };
    
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "ATMTwqP/////AAABmbJZDQrx302NndMyOIh0wdhCgAs4qfPWCBF66qx8jDGtA1RaCQUF/nQdK1LVD+e7V1VqVnq9qCHkKRXwAnpsHT+evkaZECFu4mj3lxNFaCT91Cx6fHGzKer7kUE+7YGj+Cf5fIJqCgxB1rR+FMGRHOTBnnMojuDq/gZnyW5mJVWk/XepDHJiU51AhUSd8hvthylxaXSF5Cbpnowx7BMYHOYYA0SNWt9KgPEOjk6VblPoDzcbJjsVEvI55ZSLk3FphhWH0iBsBT7+fJzd5hHjlt+L99erhiExlAfn3FnMRoBqeSSWLWFtAnjoGvFy6upt556ziiGARTxjQGtCW3Pec6Tt5LMDPznLvIwbGUxQnU3m";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;

    // Class Members
    static public OpenGLMatrix lastLocation   = null;
    static public VuforiaTrackables targets   = null ;
    static List<VuforiaTrackable> allTrackables = null;
    static private boolean targetVisible       = false;
    static public VuforiaCurrentGame vuforiaPOWERPLAY = null;
    
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    static public VuforiaLocalizer vuforia;

    /**
     * Variables used for switching cameras.
     */
    static private WebcamName webcam1, webcam2;
    static private SwitchableCamera switchableCamera;
    static private boolean oldLeftBumper;
    static private boolean oldRightBumper;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    static private TFObjectDetector tfod;
    
    static String curTilePosition;  // String with format of "Column" + "Row"

    // Vuforia names for all of our Trackable images on the field
    static final String constantTrackableB1 = "Blue Audience Wall";
    static final String constantTrackableB6 = "Blue Rear Wall";
    static final String constantTrackableE1 = "Red Audience Wall";
    static final String constantTrackableE6 = "Red Rear Wall";
  
    // Handles for each of our webcams
    static final String constantWebcam1 = "Webcam 1";
    static final String constantWebcam2 = "Webcam 2";

    static String camViewFront;
    static String camViewSide;

    static int ParkingLocationOfLabel;

    // Initialize our Lists for configuration settings
    static List<String> optionsInitialTilePosition; 
    static List<String> optionsTrackable;
    static List<String> optionsCamViewFront;
    static List<String> optionsCamViewSide;
    
  //============================================================================

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
    telemetry.addData("Current Tile Position", curTilePosition);
 
  } // end method initVisionAI()

  @ExportToBlocks (
    heading = "Vision AI",
    color = 32,
    comment = "Activate Vuforia.",
    tooltip = "Assumes Vuforia has been initialized."
  )
   /** Activate Vuforia
    */
  static public boolean activateVuforia(){
    vuforiaPOWERPLAY.activate();
    return true;
  }  // end method activateVuforia() 

  @ExportToBlocks (
    heading = "Vision AI",
    color = 32,
    comment = "Deactivate Vuforia.",
    tooltip = "Assumes Vuforia is active."
  )
   /** Deactivate Vuforia
    */
  static public boolean deactivateVuforia(){
    vuforiaPOWERPLAY.deactivate();
    return false;
  }  // end method deactivateVuforia() 

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

    @ExportToBlocks (
      heading = "Trackables",
      color = 32,
      comment = "Activate tracking the Vuforia Trackable targets.",
      tooltip = "Assumes Vuforia has been initialized."
    )
    /**
     * Activate tracking the Vuforia Trackable targets
     **/
    static public boolean activateTrackables(){

      if (targets != null) {
        targets.activate();
      } else {
        return false;
      }
      
      return true;

    }  // end method activateTrackables() 


    @ExportToBlocks (
      heading = "Trackables",
      color = 32,
      comment = "Deactivate tracking of Vuforia Trackable targets.",
      tooltip = "Assumes Vuforia is tracking."
    )
    /**
     * Deactivate tracking the Vuforia Trackable targets
     **/
    static public boolean deactivateTrackables(){

      if (targets != null) {
        targets.deactivate();
      } else {
        return false;
      }
      
      return false;

    }  // end method deactivateTrackables() 
    

    @ExportToBlocks (
      heading = "Trackables",
      color = 32,
      comment = "Detect targets visible.",
      tooltip = "Assumes tracking has been activated."
    )
    /**
     * Detects Vuforia Trackable targets
     **/
    static public boolean isTrackableVisible(){
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        } else {
            telemetry.addData("Visible Target", "none");
        }
        
        telemetry.update();
        
        return targetVisible;
        
    }  // end method targetVisible() 


    /***
     * Identify a target by naming it, and setting its position and orientation on the field
     * @param targetIndex
     * @param targetName
     * @param dx, dy, dz  Target offsets in x,y,z axes
     * @param rx, ry, rz  Target rotations in x,y,z axes
     */
    static void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);        

        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));

    }

    @ExportToBlocks (
      heading = "Vision AI",
      color = 32,
      comment = "Activate Tensorflow.",
      tooltip = "Assumes Tensorflow has been initialized."
    )
    /**
     * Activate TensorFlow Object Detection before we wait for the start command.
     * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
     **/
    static public boolean activateTensorflow(){

      if (tfod != null) {
        tfod.activate();

        // The TensorFlow software will scale the input images from the camera to a lower resolution.
        // This can result in lower detection accuracy at longer distances (> 55cm or 22").
        // If your target is at distance greater than 50 cm (20") you can increase the magnification value
        // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
        // should be set to the value of the images used to create the TensorFlow Object Detection model
        // (typically 16/9).
        tfod.setZoom(1.0, 16.0/9.0);
      } else {
        return false;
      }
      
      return true;

    }  // end method activateTensorflow() 

    @ExportToBlocks (
      heading = "Parking Signal",
      color = 32,
      comment = "Looks through all the object recognitions and determine which parking location it correlates to, if any." +
                "The parking location are for a custom signal tensorflow model.",
       tooltip = "Returns [1,2,3] or 0 if did not recognize any signals in the line of sight."
    )
    /** Runs through all the object recognitions
     *  Compares an object label from our known custom signal labels
     *  Returns the Parking Location if Signal found.  Otherwise, returns 0.
     */
    static public String identifyParkingLocationFromCustomSignal() {
        String parkingLocation = "";
        
        if (tfod != null) {
            doCameraSwitching();  // Detects left or right bumpers to switch cameras
            List<Recognition> recognitions = tfod.getRecognitions();
            telemetry.addData("# Objects Detected", recognitions.size());
            
            // step through the list of recognitions and display image size and position
            // Note: "Image number" refers to the randomized image orientation/number
            for (Recognition recognition : recognitions) {
                double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                parkingLocation = recognition.getLabel();
                
                telemetry.addData(""," ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
            }

        } 
        
        return parkingLocation;
        
    }  // end method identifyParkingLocationFromCustomSignal()


    @ExportToBlocks (
      heading = "Vision AI",
      color = 32,
      comment = "Initialize Vuforia with switchable web cameras",
      tooltip = "Initialize Vuforia with switchable web cameras"
    )
    /**
     * Initialize the Vuforia localization engine with switchable web cameras
     */
    static public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC screen);
         * If no camera-preview is desired, use the parameter-less constructor instead (commented out below).
         * Note: A preview window is required if you want to view the camera stream on the Driver Station Phone.
         */
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        vuforiaPOWERPLAY = new VuforiaCurrentGame();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // Indicate that we wish to be able to switch cameras.
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        parameters.cameraName = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        // Initialize Vuforia using SwitchableCamera
        vuforiaPOWERPLAY.initialize(
        parameters.vuforiaLicenseKey, // vuforiaLicenseKey
        parameters.cameraName, // cameraName
        "", // webcamCalibrationFilename
        parameters.useExtendedTracking, // useExtendedTracking
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
        
        vuforia = vuforiaPOWERPLAY.getVuforiaLocalizer();

        // Set the active camera to Webcam 1.
        switchableCamera = (SwitchableCamera) vuforiaPOWERPLAY.getVuforiaLocalizer().getCamera();
        switchableCamera.setActiveCamera(webcam1);
        
        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targets = (VuforiaTrackables) vuforiaPOWERPLAY.getVuforiaLocalizer().loadTrackablesFromAsset("PowerPlay");
        
        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Name and locate each trackable object
        identifyTarget(0, "Red Audience Wall",   -halfField,  -oneAndHalfTile, mmTargetHeight, 90, 0,  90);
        identifyTarget(1, "Red Rear Wall",        halfField,  -oneAndHalfTile, mmTargetHeight, 90, 0, -90);
        identifyTarget(2, "Blue Audience Wall",  -halfField,   oneAndHalfTile, mmTargetHeight, 90, 0,  90);
        identifyTarget(3, "Blue Rear Wall",       halfField,   oneAndHalfTile, mmTargetHeight, 90, 0, -90);

        /*
         * Create a transformation matrix describing where the camera is on the robot.
         *
         * Info:  The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * For a WebCam, the default starting orientation of the camera is looking UP (pointing in the Z direction),
         * with the wide (horizontal) axis of the camera aligned with the X axis, and
         * the Narrow (vertical) axis of the camera aligned with the Y axis
         *
         * But, this example assumes that the camera is actually facing forward out the front of the robot.
         * So, the "default" camera position requires two rotations to get it oriented correctly.
         * 1) First it must be rotated +90 degrees around the X axis to get it horizontal (its now facing out the right side of the robot)
         * 2) Next it must be be rotated +90 degrees (counter-clockwise) around the Z axis to face forward.
         *
         * Finally the camera can be translated to its actual mounting position on the robot.
         *      In this example, it is centered on the robot (left-to-right and front-to-back), and 6 inches above ground level.
         */

        final float CAMERA_FORWARD_DISPLACEMENT  = 0.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        /**  Let all the trackable listeners know where the camera is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }
        
    }  // end method initVuforia()

    @ExportToBlocks (
      heading = "Vision AI",
      color = 32,
      comment = "Currently using for custom Parking Signal.",
      tooltip = "Currently using for custom Parking Signal."
    )
    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    static public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }  // end method initTfod()

    @ExportToBlocks (
      heading = "Vision AI",
      color = 32,
      comment = "Switches between a Front and Side camera.",
      tooltip = "Uses bumpers to switch the cameras."
    )
    /** Switches between a Front and Side camera. 
     */
    static public void doCameraSwitching() {
        // If the left bumper is pressed, use Webcam 1.
        // If the right bumper is pressed, use Webcam 2.
        boolean newLeftBumper = gamepad1.left_bumper;
        boolean newRightBumper = gamepad1.right_bumper;
        if (newLeftBumper && !oldLeftBumper) {
            switchableCamera.setActiveCamera(webcam1);
        } else if (newRightBumper && !oldRightBumper) {
            switchableCamera.setActiveCamera(webcam2);
        }
        oldLeftBumper = newLeftBumper;
        oldRightBumper = newRightBumper;

        if (switchableCamera.getActiveCamera().equals(webcam1)) {
            telemetry.addData("activeCamera", "Webcam 1");
            telemetry.addData("Press RightBumper", "to switch to Webcam 2");
        } else {
            telemetry.addData("activeCamera", "Webcam 2");
            telemetry.addData("Press LeftBumper", "to switch to Webcam 1");
        }
    }  // end method doCameraSwitching()
    
}  // end class VisionAI


