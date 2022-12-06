package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

// For IMU
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class DrivetrainMecanum extends BlocksOpModeCompanion {
    
    // Variables for the hardware configuration names of our drive motors
    static String _driveLeftFrontName, _driveLeftBackName, _driveRightFrontName, _driveRightBackName;
    
    // Initialize our speed levels
    // powerLevel: Range 0-1.00
    static double[] powerLevel = {0.25, 0.50};
    static double powerThreshold = 0.20;
    static double turningPowerLevel = 0.25;

    // Initialize our current power / speed to zero
    static int indexPowerLevel = 0;
    static double powerAbsolute;
    
    // Variables to make sure we don't change the power level too fast
    static double powerLevelElapsedTime = 0;
    static int powerLevelChangeAllowableTime = 500;
    static ElapsedTime powerLevelChangeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    static boolean moving = false;

    // Define our motor variables
    static DcMotor driveLeftFront, driveLeftBack, driveRightFront, driveRightBack;
    
    // Define IMU
    static BNO055IMU imu;
    static BNO055IMU.Parameters imuParameters;
    static Orientation angles;
    static Acceleration gravity;

    @ExportToBlocks (
        heading = "Drivetrain",
        color = 255,
        comment = "Initialize Drivetrain and Pose",
        tooltip = "Initialize Drivetrain and Pose",
        parameterLabels = {"Left Front Drive Motor Name",
                           "Left Back Drive Motor Name",
                           "Right Front Drive Motor Name",
                           "Right Back Drive Motor Name"}
    )
    /** Initialize drivetrain and pose
     */
    public static void initDriveTrain(String driveLeftFrontName, String driveLeftBackName, String driveRightFrontName, String driveRightBackName) {
       
       // Initialize our drive train motors
       initDrivetrainMotors(driveLeftFrontName, driveLeftBackName, driveRightFrontName, driveRightBackName);
       
       // Initialize our IMU for pose information
       initIMU();

    }
    
    
    @ExportToBlocks (
        heading = "Drivetrain: Motors",
        color = 255,
        comment = "Initialize variables for our drivetrain motors.",
        tooltip = "Initialize variables for our drivetrain motors.",
        parameterLabels = {"Left Front Drive Motor Name",
                           "Left Back Drive Motor Name",
                           "Right Front Drive Motor Name",
                           "Right Back Drive Motor Name"}
    )
    /** Initialize variables for our drivetrain motors:
     *    > Current speed
     *    > Set next power threshold
     *    > Motor direction
     *    > Resetting encoder values
     *    > Set run mode of motors to not use encoders
     *    > Set motors to brake if no power
     */
    public static void initDrivetrainMotors(String driveLeftFrontName, String driveLeftBackName, String driveRightFrontName, String driveRightBackName) {

        // Save the hardware configuration names of our drive motors
        _driveLeftFrontName = driveLeftFrontName;
        _driveLeftBackName = driveLeftBackName;
        _driveRightFrontName = driveRightFrontName;
        _driveRightBackName = driveRightBackName;
        
        // Set our drive to the default X configuration
        setDriveToXConfig();
        // All drive motor handles below were set in the setDriveToXConfig() method
        
        // Resetting encoders for all drive motors
        driveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Setting run mode to not use encoders
        driveLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Setting motors to brake if no power
        driveLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Let's start our timer that checks when the last power level change occurred
        powerLevelChangeTimer.reset();
        powerLevelElapsedTime = powerLevelChangeTimer.milliseconds();
        telemetry.addData("Power Change Timer:", powerLevelElapsedTime);
    
    }  // end method initDrivetrainMotors()

    @ExportToBlocks (
        heading = "Drivetrain: Pose",
        color = 255,
        comment = "Initialize IMU variables.",
        tooltip = "Initialize IMU variables."
    )
    /** Initialize IMU and its parameters:
     *    > Angles
     *    > Gravity
     */
    public static void initIMU() {

      imu = hardwareMap.get(BNO055IMU.class, "imu");
      // Create new IMU Parameters object.
      imuParameters = new BNO055IMU.Parameters();
      
      // Use degrees as angle unit.
      imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
      // Express acceleration as m/s^2.
      imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
      // Disable logging.
      imuParameters.loggingEnabled = false;

      // Initialize IMU.
      imu.initialize(imuParameters);
    
    }  // end method initIMU()

    @ExportToBlocks (
        heading = "Drivetrain: Pose",
        color = 255,
        comment = "Get latest IMU angles.",
        tooltip = "Get latest IMU angles."
    )
    /** Get latest IMU angles
     */
    public static Orientation getLatestIMUAngles() {
      // Get latest IMU value
      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    
      setToDisplayOrientation(angles);
      
      return angles; 
      
    }  // end method getLatestIMUAngles()
    
    @ExportToBlocks (
        heading = "Drivetrain: Pose",
        color = 255,
        comment = "Set to Display IMU angles at the next telemetry update.",
        tooltip = "Need to call telemetry update."
    )
    /**
    * Sets to display the IMU angles at the next telemetry update
    */
    public static void setToDisplayOrientation(Orientation angles) {
      telemetry.addData("rot about Z", angles.firstAngle);
      telemetry.addData("rot about Y", angles.secondAngle);
      telemetry.addData("rot about X", angles.thirdAngle);
    } // end method setToDisplayOrientation()
    
    @ExportToBlocks (
        heading = "Drivetrain: Pose",
        color = 255,
        comment = "Get latest IMU gravity.",
        tooltip = "Get latest IMU gravity."
    )
    /** Get latest IMU gravity
     */
    public static Acceleration getLatestIMUGravity() {
      // Get latest IMU value
      gravity = imu.getGravity();
      
      setToDisplayGravitationalAcceleration(gravity);
      
      return gravity;
      
    }   // end method getLatestIMUGravity()
    
    @ExportToBlocks (
        heading = "Drivetrain: Pose",
        color = 255,
        comment = "Set to display IMU gravity at the next telemetry update.",
        tooltip = "Need to call telemetry update."
    )
    /**
     * Sets to display the IMU gravity at the next telemetry update
     */
   public static void setToDisplayGravitationalAcceleration(Acceleration gravity) {
     // Get acceleration due to force of gravity.
     telemetry.addData("gravity (Z)", gravity.zAccel);
     telemetry.addData("gravity (Y)", gravity.yAccel);
     telemetry.addData("gravity (X)", gravity.xAccel);
   } // end method setToDisplayGravitationalAcceleration()

    @ExportToBlocks (
        heading = "Drivetrain: Information",
        color = 255,
        comment = "Add Drivetrain info to Telemetry",
        tooltip = "Add Drivetrain info to Telemetry",
        parameterLabels = { }
    )
    /** 
     * Key data to share in the next telemetry update
     */
    public static void setToDisplayDrivetrainInfo() {
        
        telemetry.addData("current Power Level", powerLevel[indexPowerLevel]);
        telemetry.addData("encoder FrontLeft", driveLeftFront.getCurrentPosition());
        telemetry.addData("encoder FrontRight", driveRightFront.getCurrentPosition());
        telemetry.addData("encoder BackLeft", driveLeftBack.getCurrentPosition());
        telemetry.addData("encoder BackRight", driveRightBack.getCurrentPosition());
         
    }   // end method setToDisplayDrivetrainInfo()

    @ExportToBlocks (
        heading = "Drivetrain: Configurations",
        color = 255,
        comment = "Set drive to X Config.",
        tooltip = "Set drive to X Config."
    )
    /** Set our drive movements to the X, A, B, Y layout
     *  Where the button position on the gamepad relates 
     *  to the cone opening in our chassis
     *  This function is for the X layout and is the default
     */
    public static void setDriveToXConfig() {
        
        // Counterclockwise rotation in naming and for alternating positions
        driveLeftFront = hardwareMap.get(DcMotor.class, _driveLeftFrontName);
        driveLeftBack = hardwareMap.get(DcMotor.class, _driveLeftBackName);
        driveRightBack = hardwareMap.get(DcMotor.class, _driveRightBackName);
        driveRightFront = hardwareMap.get(DcMotor.class, _driveRightFrontName);

        // Reverse direction for our physically inverted motors
        driveLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        driveLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Explicitly set forward direction for our other motors
        driveRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        driveRightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        
    } // end method setDriveToXConfig()
    
    @ExportToBlocks (
        heading = "Drivetrain: Configurations",
        color = 255,
        comment = "Set drive to A Config.",
        tooltip = "Set drive to A Config."
    )
    /** Set our drive movements to the X, A, B, Y layout
     *  Where the button position on the gamepad relates 
     *  to the cone opening in our chassis
     *  This function is for the A layout
     */
    public static void setDriveToAConfig() {
        
        // Counterclockwise rotation in naming and for alternating positions
        driveLeftFront = hardwareMap.get(DcMotor.class, _driveRightFrontName);
        driveLeftBack = hardwareMap.get(DcMotor.class, _driveLeftFrontName);
        driveRightBack = hardwareMap.get(DcMotor.class, _driveLeftBackName);
        driveRightFront = hardwareMap.get(DcMotor.class, _driveRightBackName);

        // Set the direction appropriately for expected movements
        driveLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        driveRightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        driveRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        driveLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        
    } // end method setDriveToAConfig()
    
    @ExportToBlocks (
        heading = "Drivetrain: Configurations",
        color = 255,
        comment = "Set drive to B Config.",
        tooltip = "Set drive to B Config."
    )
    /** Set our drive movements to the X, A, B, Y layout
     *  Where the button position on the gamepad relates 
     *  to the cone opening in our chassis
     *  This function is for the B layout
     */
    public static void setDriveToBConfig() {
        
        // Counterclockwise rotation in naming and for alternating positions
        driveLeftFront = hardwareMap.get(DcMotor.class, _driveRightBackName);
        driveLeftBack = hardwareMap.get(DcMotor.class, _driveRightFrontName);
        driveRightBack = hardwareMap.get(DcMotor.class, _driveLeftFrontName);
        driveRightFront = hardwareMap.get(DcMotor.class, _driveLeftBackName);

        // Set the direction appropriately for expected movements
        driveLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        driveRightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        driveRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        driveLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        
    } // end method setDriveToBConfig()
    
    @ExportToBlocks (
        heading = "Drivetrain: Configurations",
        color = 255,
        comment = "Set drive to Y Config.",
        tooltip = "Set drive to Y Config."
    )
    /** Set our drive movements to the X, A, B, Y layout
     *  Where the button position on the gamepad relates 
     *  to the cone opening in our chassis
     *  This function is for the Y layout
     */
    public static void setDriveToYConfig() {
        
        // Counterclockwise rotation in naming and for alternating positions
        driveLeftFront = hardwareMap.get(DcMotor.class, _driveLeftBackName);
        driveLeftBack = hardwareMap.get(DcMotor.class, _driveRightBackName);
        driveRightBack = hardwareMap.get(DcMotor.class, _driveRightFrontName);
        driveRightFront = hardwareMap.get(DcMotor.class, _driveLeftFrontName);

        // Set the direction appropriately for expected movements
        driveLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        driveLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        driveRightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        driveRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        
    } // end method setDriveToYConfig()
    
    @ExportToBlocks (
        heading = "Drivetrain: Movement",
        color = 255,
        comment = "Stops drivetrain movement",
        tooltip = "Stops drivetrain movement",
        parameterLabels = { }
    )
    /**
     * Stops all drivetrain movement
     */
    public static void stopDrivetrain() {
        driveRightFront.setPower(0);
        driveLeftBack.setPower(0);
        driveLeftFront.setPower(0);
        driveRightBack.setPower(0);
        
        telemetry.addData("Drivetrain Movement", "NOT MOVING");
        moving = false;
    }   // end method stopDrivetrain()

    @ExportToBlocks (
        heading = "Drivetrain: Movement",
        color = 255,
        comment = "Moves robot forward",
        tooltip = "Moves robot forward",
        parameterLabels = { }
    )
    /**
     * Moves the robot forward at the current power level
     */
    public static boolean moveForward() {

        if (getPowerLevel() >= powerThreshold) {
            // Setting power to opposing wheels
            driveLeftFront.setPower(powerLevel[indexPowerLevel]);
            driveRightBack.setPower(powerLevel[indexPowerLevel]);
            driveRightFront.setPower(powerLevel[indexPowerLevel]);
            driveLeftBack.setPower(powerLevel[indexPowerLevel]);
            
            moving = true;
            telemetry.addData("Drivetrain Movement", "Moving FORWARD");
        } else {
            // Not enough power requested over threshold
            // Therefore, stopping
            stopDrivetrain();
            moving = false;
        }
        return moving;
    }   // end method moveForward()

    @ExportToBlocks (
        heading = "Drivetrain: Movement",
        color = 255,
        comment = "Moves robot backward",
        tooltip = "Moves robot backward",
        parameterLabels = {}
    )
    /**
     * Moves the robot backward at the current power level
     */
    public static boolean moveBackward() {

         if (getPowerLevel() >= powerThreshold) {
             
            // Setting power to opposing wheels
            driveLeftFront.setPower(-powerLevel[indexPowerLevel]);
            driveRightBack.setPower(-powerLevel[indexPowerLevel]);
            driveRightFront.setPower(-powerLevel[indexPowerLevel]);
            driveLeftBack.setPower(-powerLevel[indexPowerLevel]);
            
            moving = true;
            telemetry.addData("Drivetrain Movement", "Moving BACKWARD");
        } else {
            // Not enough power requested over threshold
            // Therefore, stopping
            stopDrivetrain();
            moving = false;
        }
        return moving;
    }   // end method moveBackward()

    @ExportToBlocks (
        heading = "Drivetrain: Movement",
        color = 255,
        comment = "Moves robot left",
        tooltip = "Moves robot left",
        parameterLabels = {}
    )
    /**
     * Moves robot left at current power level
     */
    public static boolean moveLeft() {
         
         if (getPowerLevel() >= powerThreshold) {
            // Setting power to opposing wheels
            // Setting first set of opposing wheels to positive
            driveRightFront.setPower(powerAbsolute);
            driveLeftBack.setPower(powerAbsolute);
            
            // Setting second set of opposing wheels to negative
            driveLeftFront.setPower(-powerAbsolute);
            driveRightBack.setPower(-powerAbsolute);
            
            moving = true;
            telemetry.addData("Drivetrain Movement", "Moving LEFT");
         } else {
            // Not enough power requested over threshold
            // Therefore, stopping
            stopDrivetrain();
            moving = false;
        }
        return moving;
    }   // end method moveLeft()

    @ExportToBlocks (
        heading = "Drivetrain: Movement",
        color = 255,
        comment = "Moves robot right",
        tooltip = "Moves robot right",
        parameterLabels = {}
    )
    /**
     * Moves robot right at current power level
     */
    public static boolean moveRight() {

         if (getPowerLevel() >= powerThreshold) {
            // Setting power to opposing wheels
            // Setting first set of opposing wheels to positive
            driveLeftFront.setPower(powerAbsolute);
            driveRightBack.setPower(powerAbsolute);
              
            // Setting second set of opposing wheels to negative
            driveRightFront.setPower(-powerAbsolute);
            driveLeftBack.setPower(-powerAbsolute);
            
            moving = true;
            telemetry.addData("Drivetrain Movement", "Moving RIGHT");
         } else {
            // Not enough power requested over threshold
            // Therefore, stopping
            stopDrivetrain();
            moving = false;
        }

        return moving;
    }   // end method moveRight()

    @ExportToBlocks (
        heading = "Drivetrain: Movement",
        color = 255,
        comment = "Turns robot clockwise",
        tooltip = "Turns robot clockwise",
        parameterLabels = {}
    )
    /**
     * Turns robot clockwise
     */
    public static boolean turnClockwise() {

         if (turningPowerLevel >= powerThreshold) {
             
            // Setting power to wheels on same side
            // Setting first set of same-side wheels to positive
            driveLeftFront.setPower(turningPowerLevel);
            driveLeftBack.setPower(turningPowerLevel);
            
            // Setting second set of opposing wheels to negative
            driveRightFront.setPower(-turningPowerLevel);
            driveRightBack.setPower(-turningPowerLevel);
            
            moving = true;
            telemetry.addData("Movement", "Turning CLOCKWISE");
            
        } else {
            // Not enough power requested over threshold
            // Therefore, stopping
            stopDrivetrain();
            moving = false;
        }

        return moving;
    }   // end method turnClockwise()

    @ExportToBlocks (
        heading = "Drivetrain: Movement",
        color = 255,
        comment = "Turns robot counterclockwise",
        tooltip = "Turns robot counterclockwise"
    )
    /**
     * Turns robot counterclockwise
     */
    public static boolean turnCounterClockwise() {

         if (turningPowerLevel >= powerThreshold) {
             
              // Setting power to wheels on same side
              // Setting first set of same-side wheels to positive
              driveLeftFront.setPower(-turningPowerLevel);
              driveLeftBack.setPower(-turningPowerLevel);
              
              // Setting second set of opposing wheels to negative
              driveRightFront.setPower(turningPowerLevel);
              driveRightBack.setPower(turningPowerLevel);
              
              moving = true;
              telemetry.addData("Movement", "Turning COUNTERCLOCKWISE");
         } else {
              // Not enough power requested over threshold
              // Therefore, stopping
              stopDrivetrain();
              moving = false;
         }

         return moving;
    }  // end method turnCounterClockwise()

    @ExportToBlocks (
        heading = "Drivetrain: Information",
        color = 255,
        comment = "Returns Current Power Level of Drivetrain",
        tooltip = "Returns Current Power Level of Drivetrain",
        parameterLabels = {}
    )
    /**
     * Returns current power level setting of all movements
     */
    public static double getPowerLevel() {
        powerAbsolute = Math.abs(powerLevel[indexPowerLevel]);
        return powerAbsolute;
    }  // end method getPowerLevel()

    @ExportToBlocks (
        heading = "Drivetrain: Movement",
        color = 255,
        comment = "Increases Power Level of All Movements",
        tooltip = "Increases Power Level of All Movements",
        parameterLabels = {}
    )
    /**
     * Increases Power Level of All Movements
     */
    public static double increasePowerLevel() {
        
        powerLevelElapsedTime = powerLevelChangeTimer.milliseconds();
        telemetry.addData("Power Change Timer:", powerLevelElapsedTime);
        telemetry.addData("Power Change Allowable Time:", powerLevelChangeAllowableTime);
   
        // If we have changed the power level too recently than don't change.
        // Quit our method and return the current power level
        if (powerLevelElapsedTime < powerLevelChangeAllowableTime){
            // Setting information to display
            telemetry.addData("Drivetrain Power Level", "NOT CHANGING, TOO RECENT");
            telemetry.addData("Current Power Level Index", indexPowerLevel);

            return powerLevel[indexPowerLevel];
        }
        
        // If still another power level then increase our power
        if (indexPowerLevel < powerLevel.length - 1) {
            // Currently lower than highest power level
            
            // Setting information to display
            telemetry.addData("Drivetrain Power Level", "INCREASING");

            // Increasing power level
            indexPowerLevel= indexPowerLevel + 1;

            telemetry.addData("Current Power Level Index", indexPowerLevel);

            // Resetting our timer for this power level change
            powerLevelChangeTimer.reset();
            
        } else {
            // Setting information to display
            telemetry.addData("Drivetrain Power Level", "AT MAX ALREADY");
            telemetry.addData("Current Power Level Index", indexPowerLevel);
        }
        
        return powerLevel[indexPowerLevel];
    }  // end method increasePowerLevel()

    @ExportToBlocks (
        heading = "Drivetrain: Movement",
        color = 255,
        comment = "Decreases Power Level of All Movements",
        tooltip = "Decreases Power Level of All Movements",
        parameterLabels = {}
    )
    /**
     * Decreases power level of all movements
     */
    public static double decreasePowerLevel() {

        powerLevelElapsedTime = powerLevelChangeTimer.milliseconds();
        telemetry.addData("Power Change Timer:", powerLevelElapsedTime);
        telemetry.addData("Power Change Allowable Time:", powerLevelChangeAllowableTime);
   
        // If we have changed the power level too recently than don't change.
        // Quit our method and return the current power level
        if (powerLevelElapsedTime < powerLevelChangeAllowableTime){
            // Setting information to display
            telemetry.addData("Drivetrain Power Level", "NOT CHANGING, TOO RECENT");
            telemetry.addData("Current Power Level Index", indexPowerLevel);

            return powerLevel[indexPowerLevel];
        }
        
        if (indexPowerLevel > 0) {
            // Currently higher than lowest power level
                
            // Setting information to display
            telemetry.addData("Drivetrain Power Level", "DECREASING");

            indexPowerLevel= indexPowerLevel - 1;
                
            // Setting information to display
            telemetry.addData("Current Power Level Index", indexPowerLevel);
                
            // Resetting our timer for this power level change
            powerLevelChangeTimer.reset();
            
        } else {
            // Setting information to display
            telemetry.addData("Drivetrain Power Level", "AT LOWEST ALREADY");
            telemetry.addData("Current Power Level Index", indexPowerLevel);
        }
        
        return powerLevel[indexPowerLevel];
    }   // end method decreasePowerLevel()

}   // end class DrivetrainMecanum
