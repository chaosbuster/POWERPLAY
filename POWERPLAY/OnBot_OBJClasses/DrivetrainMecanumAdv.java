package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// For IMU
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.State;

public class DrivetrainMecanumAdv extends BlocksOpModeCompanion {

    /* Declare our class variables. */
    
    // Define our motor hardware handles
    static private DcMotor  driveLeftFrontHW   = null;
    static private DcMotor  driveRightBackHW   = null;
    
    static private DcMotor  driveRightFrontHW  = null;
    static private DcMotor  driveLeftBackHW    = null;
    
    // Variables for the hardware configuration names of our drive motors
    static private String _driveLeftFrontName, _driveLeftBackName, _driveRightFrontName, _driveRightBackName;
    
    // Initialize our speed levels
    // powerLevel: Range 0-1.00
    static private double[] powerLevel = {0.15, 0.50};    
    // Initialize our current power / speed to our mid-level
    static int indexPowerLevel = 1;
    static double powerAbsolute = powerLevel[indexPowerLevel];

    static double powerThreshold = 0.15;
    static double turningPowerLevel = 0.25;
    
    // Variables to make sure we don't change the power level too fast
    static double powerLevelElapsedTime = 0;
    static int powerLevelChangeAllowableTime = 500;
    static ElapsedTime powerLevelChangeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    // TODO: Is this really needed now that we have a state machine?
    static boolean moving = false;

    static private BNO055IMU  imu         = null;      // Expansion Hub IMU
    // Create new IMU Parameters object.
    static BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();;
    static Orientation angles;
    static Acceleration gravity;

    static private double  robotHeading  = 0;
    static private double  headingOffset = 0;
    static private double  headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    static private double  targetHeading    = 0;
    static private double  targetDistance   = 0;
    static private double  driveSpeed       = 0;
    static private double  maxDriveSpeed    = 0;
    static private double  maxTurnSpeed     = 0;
    static private double  turnSpeed        = 0;
    static private double  speedLeftFront   = 0;
    static private double  speedRightBack   = 0;
    static private double  speedRightFront  = 0;
    static private double  speedLeftBack    = 0;
    static private int     targetLeftFront  = 0;
    static private int     targetRightBack  = 0;
    static private int     targetRightFront = 0;
    static private int     targetLeftBack   = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 560.0 ;   // REV HD HEX 20:1 Planetary 300 RPM 28x20 (eg: 537.7 for GoBILDA 312 RPM Yellow Jacket)
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 2.95 ;     // For figuring circumference;  75 mm REV mecanum wheels
    static final double     MECANUM_FACTOR_FORWARD = 1.0;     // Adjustment for theoretical calculations for FORWARD
    static final double     MECANUM_FACTOR_BACKWARD = 1.0;     // Adjustment for theoretical calculations for BACKWARD
    static final double     MECANUM_FACTOR_LEFT    = 1.2;     // Adjustment for theoretical calculations for LEFT 
    static final double     MECANUM_FACTOR_RIGHT    = 1.2;     // Adjustment for theoretical calculations for RIGHT 
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
                                                               // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    // Run states for our drivetrain
    // NOTE:  A target level must be set for the lift to raise or lower
    static private enum State {
      WAITING_FOR_COMMAND,
      DRIVING_STRAIGHT_TO_POSITION, 
      DRIVING_LEFT_TO_POSITION,
      TURNING_TO_HEADING,
      HOLDING_HEADING,
      MOVE_TO_OBJECT,
      MOVE_TOWARDS_OBJECT,
      MOVE_AWAY_FROM_OBJECT
    }
  
    static State drivetrainState = State.WAITING_FOR_COMMAND;
    static ElapsedTime timerOfARunState = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    static final int maxRunTime = 4000;   // Max run time for a state is 4 seconds.

    // Timer for when requested to hold a heading
    static ElapsedTime timerHoldHeading = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    static int timeToHoldHeading = 0;
    
    // Distance sensor related variables
    private static DistanceSensor sensor2MDistance;
    static double distanceCurrent = 0;
    
    static int distanceObjectSeen = 0;
    static boolean directionForward = true;
    
    static int distanceFromObject = 170;
    static int distanceToClearObject = 300;
    static boolean directionLeft = true;
    static boolean directionClockwise = true;
    static boolean previousJogForward = true;
    static final int maxRunTimeTowardsObject = 1500;   // Max run time for going towards an object
    
    static private enum Config {
      X,
      Y,
      B,
      A
    }
    
    static Config drivetrainConfig = Config.X;

    // **********  Drivetrain Initialization (or Modes)  ********************
    
    @ExportToBlocks (
        heading = "Drivetrain",
        color = 255,
        comment = "Initialize Drivetrain and Pose",
        tooltip = "Initialize Drivetrain and Pose",
        parameterLabels = {"Left Front Drive Motor Name",
                           "Left Back Drive Motor Name",
                           "Right Front Drive Motor Name",
                           "Right Back Drive Motor Name",
                           "2M Distance Sensor"
        }
    )
    /** Initialize drivetrain and pose
     */
    static public void initDriveTrain(String driveLeftFrontName, String driveLeftBackName, String driveRightFrontName, String driveRightBackName, String sensor2MDistanceName) {

       // Save the hardware configuration names of our drive motors
       _driveLeftFrontName = driveLeftFrontName;
       _driveLeftBackName = driveLeftBackName;
       _driveRightFrontName = driveRightFrontName;
       _driveRightBackName = driveRightBackName;
       
       // Initialize to Config X
       // Set our drive to the default X configuration
       setDriveToXConfig();
       
       // Initialize our drive train motors
       initDrivetrainMotors();
       
       // Let's get a hardware handle on our distance sensor at the bottom of our lift
       sensor2MDistance = hardwareMap.get(DistanceSensor.class, sensor2MDistanceName);

       // Initialize our IMU for pose information
       initIMU();
       
       resetHeading();
       
       // Let's start our timer that checks when the last power level change occurred
       powerLevelChangeTimer.reset();
       powerLevelElapsedTime = powerLevelChangeTimer.milliseconds();
       telemetry.addData("Power Change Timer:", powerLevelElapsedTime);
       

    }  // end method initDriveTrain()
     
    /** Initialize variables for our drivetrain motors:
     *    > Current speed
     *    > Set next power threshold
     *    > Motor direction
     *    > Resetting encoder values
     *    > Set run mode of motors to not use encoders
     *    > Set motors to brake if no power
     */
    static private void initDrivetrainMotors() {
        
        // All drive motor handles below were set in the setDriveToXConfig() method
        
        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode

        // Resetting encoders for all drive motors
        driveLeftFrontHW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLeftBackHW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRightFrontHW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRightBackHW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Setting motors to brake if no power
        driveLeftFrontHW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveLeftBackHW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRightFrontHW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRightBackHW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }  // end method initDrivetrainMotors()

    /** Initialize IMU and its parameters:
     *    > Angles
     *    > Gravity
     */
    static private void initIMU() {

      imu = hardwareMap.get(BNO055IMU.class, "imu");
      
      // Use degrees as angle unit.
      imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
      // Express acceleration as m/s^2.
      imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
      // Disable logging.
      imuParameters.loggingEnabled = false;

      // Initialize IMU.
      imu.initialize(imuParameters);
      
      telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
    
    }  // end method initIMU()

    @ExportToBlocks (
        heading = "Drivetrain",
        color = 255,
        comment = "Ready our drive motors to RUN_TO_POSITION.",
        tooltip = "Needs encoders on all the drive motors."
    )
    /** Set all the drive motors ready to RUN_TO_POSITION.
     */
    public static void readyRunToPosition() {

       // Set the encoders for closed loop speed control, and reset the heading.
       driveLeftFrontHW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       driveLeftBackHW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       driveRightFrontHW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       driveRightBackHW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               
    }  // end method readyRunToPosition()
    
    @ExportToBlocks (
        heading = "Drivetrain",
        color = 255,
        comment = "Ready our drive motors to run without encoders.",
        tooltip = "Does not require encoders on drive motors."
    )
    /** Set all the drive motors ready to RUN_WITHOUT_ENCODER.
     */
    public static void readyWOEncoders() {

        // Setting run mode to not use encoders
        driveLeftFrontHW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveLeftBackHW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveRightFrontHW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveRightBackHW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
               
    }  // end method readyWOEncoders()

    
        // ********  HIGH Level driving functions. (Use for TeleOP) **************

    @ExportToBlocks (
        heading = "Drivetrain: Teleop",
        color = 255,
        comment = "Stops drivetrain movement",
        tooltip = "Stops drivetrain movement",
        parameterLabels = { }
    )
    /**
     * Stops all drivetrain movement
     */
    public static void stopDrivetrain() {
        driveRightFrontHW.setPower(0);
        driveLeftBackHW.setPower(0);
        driveLeftFrontHW.setPower(0);
        driveRightBackHW.setPower(0);
        
        telemetry.addData("Drivetrain Movement", "NOT MOVING");
        moving = false;
    }   // end method stopDrivetrain()

    @ExportToBlocks (
        heading = "Drivetrain: Teleop",
        color = 255,
        comment = "Moves robot forward or backward.",
        tooltip = "Forward is true, Backward is false.",
        parameterLabels = {"Are we moving forward"}
    )
    /**
     * Moves the robot forward or backward at the current power level
     */
    public static boolean moveForward(boolean movingforward) {
        
        directionForward = movingforward;

        if (getPowerLevel() >= powerThreshold) {
            if (directionForward) {
                // Setting power to opposing wheels
                driveLeftFrontHW.setPower(powerLevel[indexPowerLevel]);
                driveRightBackHW.setPower(powerLevel[indexPowerLevel]);
                driveRightFrontHW.setPower(powerLevel[indexPowerLevel]);
                driveLeftBackHW.setPower(powerLevel[indexPowerLevel]);
                telemetry.addData("Drivetrain Movement", "Moving FORWARD");
            } else {
                // Setting power to opposing wheels
                driveLeftFrontHW.setPower(-powerLevel[indexPowerLevel]);
                driveRightBackHW.setPower(-powerLevel[indexPowerLevel]);
                driveRightFrontHW.setPower(-powerLevel[indexPowerLevel]);
                driveLeftBackHW.setPower(-powerLevel[indexPowerLevel]);
                telemetry.addData("Drivetrain Movement", "Moving BACKWARD");
            }
            moving = true;
        } else {
            // Not enough power requested over threshold
            // Therefore, stopping
            stopDrivetrain();
            moving = false;
        }
        return moving;
    }   // end method moveForward()

    @ExportToBlocks (
        heading = "Drivetrain: Teleop",
        color = 255,
        comment = "Moves robot left or right.",
        tooltip = "Left is true, Right is false.",
        parameterLabels = {"Are we moving left"}
    )
    /**
     * Moves robot left or right at current power level
     */
    public static boolean moveLeft(boolean movingLeft) {
                 
        directionLeft = movingLeft;
         
        if (getPowerLevel() >= powerThreshold) {
            if (directionLeft) {
                // MOVING LEFT
                // Setting power to opposing wheels
                // Setting first set of opposing wheels to positive
                driveRightFrontHW.setPower(powerAbsolute);
                driveLeftBackHW.setPower(powerAbsolute);
            
                // Setting second set of opposing wheels to negative
                driveLeftFrontHW.setPower(-powerAbsolute);
                driveRightBackHW.setPower(-powerAbsolute);
                
                telemetry.addData("Drivetrain Movement", "Moving LEFT");
            } else {
                // MOVING RIGHT
                // Setting power to opposing wheels
                // Setting first set of opposing wheels to positive
                driveRightFrontHW.setPower(-powerAbsolute);
                driveLeftBackHW.setPower(-powerAbsolute);
              
                // Setting second set of opposing wheels to negative
                driveLeftFrontHW.setPower(powerAbsolute);
                driveRightBackHW.setPower(powerAbsolute);
           
                telemetry.addData("Drivetrain Movement", "Moving RIGHT");
            }
            moving = true;
         } else {
            // Not enough power requested over threshold
            // Therefore, stopping
            stopDrivetrain();
            moving = false;
        }
        return moving;
    }   // end method moveLeft()

    @ExportToBlocks (
        heading = "Drivetrain: Teleop",
        color = 255,
        comment = "Turns robot",
        tooltip = "Clockwise is true, Counter-Clockwise is false.",
        parameterLabels = {"Turn clockwise"}
    )
    /**
     * Turns robot clockwise
     */
    public static void turnClockwise(boolean turningClockwise) {
        
        directionClockwise = turningClockwise;
        
        if (turningPowerLevel >= powerThreshold) {
        
            if (directionClockwise) {        
                // Setting power to wheels on same side
                // Setting first set of same-side wheels to positive
                driveLeftFrontHW.setPower(turningPowerLevel);
                driveLeftBackHW.setPower(turningPowerLevel);
            
                // Setting second set of opposing wheels to negative
                driveRightFrontHW.setPower(-turningPowerLevel);
                driveRightBackHW.setPower(-turningPowerLevel);

                telemetry.addData("Movement", "Turning CLOCKWISE");
            } else {

                // Setting power to wheels on same side
                // Setting first set of same-side wheels to positive
                driveLeftFrontHW.setPower(-turningPowerLevel);
                driveLeftBackHW.setPower(-turningPowerLevel);
              
                // Setting second set of opposing wheels to negative
                driveRightFrontHW.setPower(turningPowerLevel);
                driveRightBackHW.setPower(turningPowerLevel);
              
                telemetry.addData("Movement", "Turning COUNTERCLOCKWISE");
            }
            moving = true;
            
        } else {
            // Not enough power requested over threshold
            // Therefore, stopping
            stopDrivetrain();
            moving = false;
        }

    }   // end method turnClockwise()

    
    @ExportToBlocks (
     heading = "Drivetrain: Autonomous or Assists",
     color = 255,
     comment = "Runs drivetrain based on the state.",
     tooltip = "States are changed by calling a movement or heading."
    )
    /**
     * Runs an interation of the drivetrain based on it's current state
     * Running the drive motors in a STATE mode to allow other actions to happen on the robot.
     */
    public static void runDrivetrainIteration() {
 
    switch (drivetrainState) {
      
      case WAITING_FOR_COMMAND: {
          
        telemetry.addData("Movement: ", "WAITING_FOR_COMMAND");
        break;
      }

      case DRIVING_STRAIGHT_TO_POSITION: {
                
        telemetry.addData("Movement: ", "DRIVING STRAIGHT");
        
        // keep looping while we are still active, and BOTH motors are running.
        if (timerOfARunState.milliseconds() < maxRunTime && 
           (driveLeftFrontHW.isBusy() && driveRightBackHW.isBusy() && driveRightFrontHW.isBusy() && driveLeftBackHW.isBusy())) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(targetHeading, maxDriveSpeed, P_DRIVE_GAIN);

            // if driving in reverse, the motor correction also needs to be reversed
            if (targetDistance < 0)
                turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            moveRobotRTP(driveSpeed, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(true);
        } else {
            
            stopDrivetrainRTP();
            drivetrainState = State.WAITING_FOR_COMMAND;
        }
        
        break;
      }

      case DRIVING_LEFT_TO_POSITION: {
                
        telemetry.addData("Movement: ", "DRIVING SIDEWAYS");
        
        // keep looping while we are still active, and BOTH motors are running.
        if (timerOfARunState.milliseconds() < maxRunTime &&
           (driveLeftFrontHW.isBusy() && driveRightBackHW.isBusy() && driveRightFrontHW.isBusy() && driveLeftBackHW.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(targetHeading, maxDriveSpeed, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (targetDistance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveLeftRTP(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
        } else {    
            stopDrivetrainRTP();
            drivetrainState = State.WAITING_FOR_COMMAND;
        }

        break;
      }

      case TURNING_TO_HEADING: {
                
        telemetry.addData("Movement: ", "TURNING TO HEADING");
        
        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(targetHeading, maxTurnSpeed, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        if (timerOfARunState.milliseconds() < maxRunTime && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(targetHeading, maxTurnSpeed, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            turnRobotByHeading(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
            
        } else {
            stopDrivetrainRTP();
            drivetrainState = State.WAITING_FOR_COMMAND;
        }

        break;
      }

      case HOLDING_HEADING: {
                
        telemetry.addData("Movement: ", "HOLDING HEADING");

        // keep looping while we have time remaining.
        if (timerOfARunState.milliseconds() < maxRunTime && timerHoldHeading.milliseconds() < timeToHoldHeading) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(targetHeading, maxTurnSpeed, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            turnRobotByHeading(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
            
        } else {
            stopDrivetrainRTP();
            drivetrainState = State.WAITING_FOR_COMMAND;
        }

        break;
      }
      
      case MOVE_TO_OBJECT: {
                
        telemetry.addData("Movement: ", "MOVING FORWARD TO AN OBJECT");
        
        // keep looping while we are still active, and BOTH motors are running.
        if (timerOfARunState.milliseconds() < maxRunTime && distanceCurrent > distanceObjectSeen) {

            // Grab the latest distance reading at the bottom of the lift to see if anything there
            distanceCurrent = sensor2MDistance.getDistance(DistanceUnit.MM);

            telemetry.addData("Current distance: ", distanceCurrent);
            telemetry.addData("Look for object at distance: ", distanceObjectSeen);

            // Moving the robot
            moveForward(directionForward); 

            // Display drive status for the driver.
            sendTelemetry(true);
        } else {
            
            stopDrivetrain();
            drivetrainState = State.WAITING_FOR_COMMAND;
        }
        
        break;
      }

      case MOVE_TOWARDS_OBJECT: {
                
        telemetry.addData("Movement: ", "MOVING TOWARDS OBJECT");
        
        previousJogForward = !previousJogForward;
        
        // keep looping while we are still active, and BOTH motors are running.
        if (timerOfARunState.milliseconds() < maxRunTimeTowardsObject && distanceCurrent > distanceFromObject) {

            // Grab the latest distance reading at the bottom of the lift to see if anything there
            distanceCurrent = sensor2MDistance.getDistance(DistanceUnit.MM);
            
            // if we aren't seeing the object at all list alternate jogging forward and backward on iterations
            if (distanceCurrent > distanceObjectSeen) {
                
                if (previousJogForward) 
                    moveForward(false);
                else
                    moveForward(true);
            }

            telemetry.addData("Current distance: ", distanceCurrent);
            telemetry.addData("Look for object at distance: ", distanceFromObject);

            // Moving the robot
            moveLeft(directionLeft); 

            // Display drive status for the driver.
            sendTelemetry(true);
        } else {
            
            stopDrivetrain();
            drivetrainState = State.WAITING_FOR_COMMAND;
        }
        
        break;
      }

      case MOVE_AWAY_FROM_OBJECT: {
                
        telemetry.addData("Movement: ", "MOVING AWAY FROM OBJECT");
        
        // keep looping while we are still active, and BOTH motors are running.
        if (timerOfARunState.milliseconds() < maxRunTime && distanceCurrent < distanceToClearObject) {

            // Grab the latest distance reading at the bottom of the lift to see if anything there
            distanceCurrent = sensor2MDistance.getDistance(DistanceUnit.MM);

            telemetry.addData("Current distance: ", distanceCurrent);
            telemetry.addData("Look for object at distance: ", distanceToClearObject);

            // if we aren't seeing the object at all list alternate jogging forward and backward on iterations
            if (distanceCurrent > distanceObjectSeen) {
                
                if (previousJogForward) 
                    moveForward(false);
                else
                    moveForward(true);
            }

            // Moving the robot
            moveLeft(directionLeft); 

            // Display drive status for the driver.
            sendTelemetry(true);
        } else {
            
            stopDrivetrain();
            drivetrainState = State.WAITING_FOR_COMMAND;
        }
        
        break;
      }

    }

  }

    @ExportToBlocks (
        heading = "Drivetrain",
        color = 255,
        comment = "Returns whether we are waiting for a command.",
        tooltip = "If waiting a long time, then check the maxRunTime setting."
    )
    /** Returns whether we are waiting for a command.
     */
    public static boolean waitingForCommand() {
        if (drivetrainState == State.WAITING_FOR_COMMAND) 
            return true;
        else    
            return false;
    
    }  // end method waitingForCommand()

    @ExportToBlocks (
        heading = "Drivetrain: Information",
        color = 255,
        comment = "Returns the configuration mode.",
        tooltip = "Returns either X,B,Y or A."
    )
    /** Returns the configuration mode.
     */
    public static String currentConfiguration() {
        
        if (drivetrainConfig == Config.X) 
            return "X";
        else if (drivetrainConfig == Config.B) 
            return "B";
        else if (drivetrainConfig == Config.Y) 
            return "Y";
        else if (drivetrainConfig == Config.A) 
            return "A";
        else
            return "X";
            
    }  // end method currentConfiguration()
    


    // **********  HIGH Level driving functions.  ********************
    
    @ExportToBlocks (
        heading = "Drivetrain: Run To Position Option",
        color = 255,
        comment = "Drives Straight either Forward or Reverse.",
        tooltip = "Reverse movement is obtained by setting a negative distance (not speed).",
        parameterLabels = {"Max Drive Speed", "Distance", "Heading"}
    )
    /**
    *  Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
    * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from the current robotHeading.
    */
    static public void driveStraightRTP(double requestedMaxDriveSpeed,
                              double requestedDistance,
                              double requestedHeading) {
        
        // Friction adjustment based on direction for mecanum drive
        double adjustmentMecanum = 1.0;    
        targetDistance = requestedDistance;
        targetHeading = requestedHeading;
        
        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        maxDriveSpeed = Math.abs(requestedMaxDriveSpeed);
        
        // if driving in reverse, the motor correction also needs to be reversed
        if (targetDistance < 0)
            adjustmentMecanum = MECANUM_FACTOR_BACKWARD;
        else
            adjustmentMecanum = MECANUM_FACTOR_FORWARD;        

        // Determine new target position, and pass to motor controller
        int moveCounts = (int)(adjustmentMecanum * targetDistance * COUNTS_PER_INCH);
        targetLeftFront = driveLeftFrontHW.getCurrentPosition() + moveCounts;
        targetRightBack = driveRightBackHW.getCurrentPosition() + moveCounts;
            
        targetRightFront = driveRightFrontHW.getCurrentPosition() + moveCounts;
        targetLeftBack = driveLeftBackHW.getCurrentPosition() + moveCounts;

        // Set Target FIRST, then turn on RUN_TO_POSITION
        driveLeftFrontHW.setTargetPosition(targetLeftFront);
        driveRightBackHW.setTargetPosition(targetRightBack);

        driveRightFrontHW.setTargetPosition(targetRightFront);
        driveLeftBackHW.setTargetPosition(targetLeftBack);

        // Now RUN_TO_POSITION
        driveLeftFrontHW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveRightBackHW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
        driveRightFrontHW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveLeftBackHW.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start driving straight, and then enter the control loop
        moveRobotRTP(maxDriveSpeed, 0);

        // Set the state that was requested
        drivetrainState = State.DRIVING_STRAIGHT_TO_POSITION;
        timerOfARunState.reset();
        
        //  Our RunDrivetrainIteration() takes over from here.
        
    }   // end method driveStraightRTP()

    @ExportToBlocks (
        heading = "Drivetrain: Run To Position Option",
        color = 255,
        comment = "Drives Left or Right.",
        tooltip = "Right movement is obtained by setting a negative distance (not speed).",
        parameterLabels = {"Max Drive Speed", "Distance", "Heading"}
    )
    /**
    *  Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
    * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from the current robotHeading.
    */
    static public void driveLeftRTP(double requestedMaxDriveSpeed,
                              double requestedDistance,
                              double requestedHeading) {

        // Friction adjustment based on direction for mecanum drive
        double adjustmentMecanum = 1.0;               
        
        targetDistance = requestedDistance;
        targetHeading = requestedHeading;
        
        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        maxDriveSpeed = Math.abs(requestedMaxDriveSpeed);
        
        // if driving in reverse, the motor correction also needs to be reversed
        if (targetDistance < 0)
            adjustmentMecanum = MECANUM_FACTOR_RIGHT;
        else
            adjustmentMecanum = MECANUM_FACTOR_LEFT;        

        // Determine new target position, and pass to motor controller
        int moveCounts = (int)(adjustmentMecanum * Math.abs(targetDistance) * COUNTS_PER_INCH);
            
        if (targetDistance > 0) {
            targetLeftFront = driveLeftFrontHW.getCurrentPosition() - moveCounts;
            targetRightBack = driveRightBackHW.getCurrentPosition() - moveCounts;
            
            targetRightFront = driveRightFrontHW.getCurrentPosition() + moveCounts;
            targetLeftBack = driveLeftBackHW.getCurrentPosition() + moveCounts;
              
        } else {
                
            targetLeftFront = driveLeftFrontHW.getCurrentPosition() + moveCounts;
            targetRightBack = driveRightBackHW.getCurrentPosition() + moveCounts;
            
            targetRightFront = driveRightFrontHW.getCurrentPosition() - moveCounts;
            targetLeftBack = driveLeftBackHW.getCurrentPosition() - moveCounts;
               
        }

        // Set Target FIRST, then turn on RUN_TO_POSITION
        driveLeftFrontHW.setTargetPosition(targetLeftFront);
        driveRightBackHW.setTargetPosition(targetRightBack);

        driveRightFrontHW.setTargetPosition(targetRightFront);
        driveLeftBackHW.setTargetPosition(targetLeftBack);

        // Now RUN_TO_POSITION
        driveLeftFrontHW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveRightBackHW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
        driveRightFrontHW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveLeftBackHW.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start driving straight, and then enter the control loop
        moveLeftRTP(maxDriveSpeed, 0);
        
        // Set the state that was requested
        drivetrainState = State.DRIVING_LEFT_TO_POSITION;
        timerOfARunState.reset();
        
        //  Our RunDrivetrainIteration() takes over from here.

    }   // end method driveLeft()
    
    @ExportToBlocks (
        heading = "Drivetrain: Using Heading",
        color = 255,
        comment = "Turns to Absolute Heading Angle (in Degrees) relative to last gyro reset.",
        tooltip = "0 = fwd. +ve is CCW from fwd. -ve is CW from forward.",
        parameterLabels = {"Max Turn Speed", "Heading"}
    )
    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    static public void turnToHeading(double requestedTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, requestedTurnSpeed, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        if ((Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Set the state that was requested
            drivetrainState = State.TURNING_TO_HEADING;
            timerOfARunState.reset();
            
        } else {
            // Not within the heading threshold, so not going to set to action
            stopDrivetrainRTP();
            drivetrainState = State.WAITING_FOR_COMMAND;
        }
        
        //  Our RunDrivetrainIteration() takes over from here.
    }

    @ExportToBlocks (
        heading = "Drivetrain: Using Heading",
        color = 255,
        comment = "Obtain & hold a heading for a finite amount of time.",
        tooltip = "This function is useful for giving the robot a moment to stabilize it's heading between movements.",
        parameterLabels = {"Max Turn Speed", "Heading", "Hold Time"}
    )
    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in milliseconds) to hold the specified heading.
     */
    static public void holdHeading(double requestedTurnSpeed, double heading, int holdTime) {

        timeToHoldHeading = holdTime;
        timerHoldHeading.reset();

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, requestedTurnSpeed, P_DRIVE_GAIN);
        
        if (timerHoldHeading.milliseconds() < timeToHoldHeading) {

            // Set the state that was requested
            drivetrainState = State.HOLDING_HEADING;
            timerOfARunState.reset();
            
        } else {
            // Not within the heading threshold, so not going to set to action
            stopDrivetrainRTP();
            drivetrainState = State.WAITING_FOR_COMMAND;
        }
        
        
    }

    @ExportToBlocks (
        heading = "Drivetrain: Using Distance",
        color = 255,
        comment = "Aligns robot to an object",
        tooltip = "Robot can see the object. Pass true if moving forward",
        parameterLabels = {"Are we moving forward?", "Distance Object is Seen"}
    )
    /**
     * Moves the robot forward backward to align with the closest object
     */
    public static void moveToObject(boolean movingForward, int distanceSeen) {
        
        distanceObjectSeen = distanceSeen;
        indexPowerLevel = 0;
        directionForward = movingForward;
                
        // Grab the latest distance reading at the bottom of the lift to see if anything there
        distanceCurrent = sensor2MDistance.getDistance(DistanceUnit.MM);

        telemetry.addData("Current distance: ", distanceCurrent);
        telemetry.addData("Distance When Object Seen: ", distanceObjectSeen);
        
        // Moving the robot
        moveForward(directionForward); 

        // Set the state that was requested
        drivetrainState = State.MOVE_TO_OBJECT;
        timerOfARunState.reset();
        
        //  Our RunDrivetrainIteration() takes over from here.

    }   // end method moveToObject()


    @ExportToBlocks (
        heading = "Drivetrain: Using Distance",
        color = 255,
        comment = "Moves robot towards an object.",
        tooltip = "Will stop when at distance provided. Pass true if moving forward",
        parameterLabels = {"Are we moving left", "Distance From Object To Stop"}
    )
    /**
     * Moves the robot left or right towards an object
     */
    public static void moveTowardsObject(boolean movingLeft, int distanceToStop) {
        
        distanceFromObject = distanceToStop;
        directionLeft = movingLeft;
        // Set our speed to the slowest for accuracy                
        indexPowerLevel = 0;

        // Grab the latest distance reading at the bottom of the lift to see if anything there
        distanceCurrent = sensor2MDistance.getDistance(DistanceUnit.MM);

        telemetry.addData("Current distance: ", distanceCurrent);
        telemetry.addData("Distance To Stop from Object: ", distanceFromObject);

        // Moving the robot
        moveLeft(directionLeft); 

        // Set the state that was requested
        drivetrainState = State.MOVE_TOWARDS_OBJECT;
        timerOfARunState.reset();
        
        //  Our RunDrivetrainIteration() takes over from here.

    }   // end method moveTowardsObject()


    @ExportToBlocks (
        heading = "Drivetrain: Using Distance",
        color = 255,
        comment = "Moves back away from an object",
        tooltip = "Robot can see the object. Pass true if moving left",
        parameterLabels = {"Are we moving left", "Distance To Clear Object"}
    )
    /**
     * Moves the robot away from an object
     */
    public static void moveAwayFromObject(boolean movingLeft, int distanceToClear) {
        
        distanceToClearObject = distanceToClear;
        directionLeft = movingLeft;
        // Set our speed to the slowest for accuracy                
        indexPowerLevel = 0;

        // Grab the latest distance reading at the bottom of the lift to see if anything there
        distanceCurrent = sensor2MDistance.getDistance(DistanceUnit.MM);

        telemetry.addData("Current distance: ", distanceCurrent);
        telemetry.addData("Current distance: ", distanceToClearObject);

        // Moving the robot
        moveLeft(directionLeft); 

        // Set the state that was requested
        drivetrainState = State.MOVE_AWAY_FROM_OBJECT;
        timerOfARunState.reset();
        
        //  Our RunDrivetrainIteration() takes over from here.

    }   // end method moveAwayFromObject()


    // **********  Powerlevel Changes (Used for TeleOp).  ********************

    @ExportToBlocks (
        heading = "Drivetrain: Powerlevel",
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
        heading = "Drivetrain: Powerlevel",
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
        heading = "Drivetrain: Powerlevel",
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


    // **********  Telemetry Group Information Displays  ********************
    
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
        heading = "Drivetrain: Encoder Info",
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
        telemetry.addData("encoder FrontLeft", driveLeftFrontHW.getCurrentPosition());
        telemetry.addData("encoder FrontRight", driveRightFrontHW.getCurrentPosition());
        telemetry.addData("encoder BackLeft", driveLeftBackHW.getCurrentPosition());
        telemetry.addData("encoder BackRight", driveRightBackHW.getCurrentPosition());
         
    }   // end method setToDisplayDrivetrainInfo()


    // **********  Drivetrain Configurations  ********************
    
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
    static public void setDriveToXConfig() {
        
        // Counterclockwise rotation in naming and for alternating positions
        driveLeftFrontHW = hardwareMap.get(DcMotor.class, _driveLeftFrontName);
        driveLeftBackHW = hardwareMap.get(DcMotor.class, _driveLeftBackName);
        driveRightBackHW = hardwareMap.get(DcMotor.class, _driveRightBackName);
        driveRightFrontHW = hardwareMap.get(DcMotor.class, _driveRightFrontName);

        // Reverse direction for our physically inverted motors
        driveLeftFrontHW.setDirection(DcMotorSimple.Direction.REVERSE);
        driveLeftBackHW.setDirection(DcMotorSimple.Direction.REVERSE);

        // Explicitly set forward direction for our other motors
        driveRightFrontHW.setDirection(DcMotorSimple.Direction.FORWARD);
        driveRightBackHW.setDirection(DcMotorSimple.Direction.FORWARD);
        
        // Set our configuration state
        drivetrainConfig = Config.X;
        
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
    static public void setDriveToAConfig() {
        
        // Counterclockwise rotation in naming and for alternating positions
        driveLeftFrontHW = hardwareMap.get(DcMotor.class, _driveRightFrontName);
        driveLeftBackHW = hardwareMap.get(DcMotor.class, _driveLeftFrontName);
        driveRightBackHW = hardwareMap.get(DcMotor.class, _driveLeftBackName);
        driveRightFrontHW = hardwareMap.get(DcMotor.class, _driveRightBackName);

        // Set the direction appropriately for expected movements
        driveLeftBackHW.setDirection(DcMotorSimple.Direction.REVERSE);
        driveRightBackHW.setDirection(DcMotorSimple.Direction.FORWARD);

        driveRightFrontHW.setDirection(DcMotorSimple.Direction.FORWARD);
        driveLeftFrontHW.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Set our configuration state
        drivetrainConfig = Config.A;
        
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
    static public void setDriveToBConfig() {
        
        // Counterclockwise rotation in naming and for alternating positions
        driveLeftFrontHW = hardwareMap.get(DcMotor.class, _driveRightBackName);
        driveLeftBackHW = hardwareMap.get(DcMotor.class, _driveRightFrontName);
        driveRightBackHW = hardwareMap.get(DcMotor.class, _driveLeftFrontName);
        driveRightFrontHW = hardwareMap.get(DcMotor.class, _driveLeftBackName);

        // Set the direction appropriately for expected movements
        driveLeftBackHW.setDirection(DcMotorSimple.Direction.REVERSE);
        driveRightBackHW.setDirection(DcMotorSimple.Direction.FORWARD);

        driveRightFrontHW.setDirection(DcMotorSimple.Direction.FORWARD);
        driveLeftFrontHW.setDirection(DcMotorSimple.Direction.REVERSE);
 
         // Set our configuration state
        drivetrainConfig = Config.B;
        
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
    static public void setDriveToYConfig() {
        
        // Counterclockwise rotation in naming and for alternating positions
        driveLeftFrontHW = hardwareMap.get(DcMotor.class, _driveLeftBackName);
        driveLeftBackHW = hardwareMap.get(DcMotor.class, _driveRightBackName);
        driveRightBackHW = hardwareMap.get(DcMotor.class, _driveRightFrontName);
        driveRightFrontHW = hardwareMap.get(DcMotor.class, _driveLeftFrontName);

        // Set the direction appropriately for expected movements
        driveLeftBackHW.setDirection(DcMotorSimple.Direction.REVERSE);
        driveLeftFrontHW.setDirection(DcMotorSimple.Direction.REVERSE);

        driveRightBackHW.setDirection(DcMotorSimple.Direction.FORWARD);
        driveRightFrontHW.setDirection(DcMotorSimple.Direction.FORWARD);
        
        // Set our configuration state
        drivetrainConfig = Config.Y;
        
    } // end method setDriveToYConfig()

    // **********  LOW Level driving functions.  ********************
   
    /**
     * Stops all drivetrain movement
     */
    static public void stopDrivetrainRTP() {
        
        // Stop all motion & Turn off RUN_TO_POSITION
        moveRobotRTP(0, 0);
        driveLeftFrontHW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRightBackHW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
        driveRightFrontHW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveLeftBackHW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        telemetry.addData("Movement: ", "NOT MOVING");
        
    }   // end method stopDrivetrainRTP()

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    static public double getSteeringCorrection(double desiredHeading, double desiredTurnSpeed, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry
        maxTurnSpeed = desiredTurnSpeed; 

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    static public void moveRobotRTP(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        speedLeftFront  = drive - turn;
        speedRightBack  = drive - turn;

        speedRightFront = drive + turn;
        speedLeftBack = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.max(Math.abs(speedLeftFront), Math.abs(speedRightBack)), Math.max(Math.abs(speedRightFront), Math.abs(speedLeftBack)));
        if (max > 1.0)
        {
            speedLeftFront /= max;
            speedRightBack /= max;
            
            speedRightFront /= max;
            speedLeftBack /= max;
        }

        driveLeftFrontHW.setPower(speedLeftFront);
        driveRightBackHW.setPower(speedRightBack);

        driveRightFrontHW.setPower(speedRightFront);
        driveLeftBackHW.setPower(speedLeftBack);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    //  ADDED FOR MECANUM By Coach Breton
    static public void moveLeftRTP(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        speedLeftFront  = drive + turn;
        speedRightBack  = drive + turn;

        speedRightFront = drive - turn;
        speedLeftBack = drive - turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.max(Math.abs(speedLeftFront), Math.abs(speedRightBack)), Math.max(Math.abs(speedRightFront), Math.abs(speedLeftBack)));
        if (max > 1.0)
        {
            speedLeftFront /= max;
            speedRightBack /= max;
            
            speedRightFront /= max;
            speedLeftBack /= max;
        }

        driveLeftFrontHW.setPower(speedLeftFront);
        driveRightBackHW.setPower(speedRightBack);

        driveRightFrontHW.setPower(speedRightFront);
        driveLeftBackHW.setPower(speedLeftBack);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    //  ADDED FOR MECANUM By Coach Breton
    static public void turnRobotByHeading(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        // Turning by setting same speed for wheels on the same sides
        speedLeftFront  = drive - turn;
        speedLeftBack  = drive - turn;

        speedRightFront = drive + turn;
        speedRightBack = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.max(Math.abs(speedLeftFront), Math.abs(speedRightBack)), Math.max(Math.abs(speedRightFront), Math.abs(speedLeftBack)));
        if (max > 1.0)
        {
            speedLeftFront /= max;
            speedRightBack /= max;
            
            speedRightFront /= max;
            speedLeftBack /= max;
        }

        // Turning by setting same speed for wheels on the same sides
        driveLeftFrontHW.setPower(speedLeftFront);
        driveLeftBackHW.setPower(speedLeftBack);

        driveRightFrontHW.setPower(speedRightFront);
        driveRightBackHW.setPower(speedRightBack);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    static private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos LF:RB ,  RF:LB",  "%7d:%7d  ,  %7d:%7d", targetLeftFront, targetRightBack, targetRightFront, targetLeftBack);
            telemetry.addData("Actual Pos LF:RB ,  RF:LB",  "%7d:%7d  ,  %7d:%7d",  
                               driveLeftFrontHW.getCurrentPosition(), driveRightBackHW.getCurrentPosition(),
                               driveRightFrontHW.getCurrentPosition(), driveLeftBackHW.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds  LF:RB ,  RF:LB.", "%5.2f : %5.2f  ,  %5.2f : %5.2f", speedLeftFront, speedRightBack, speedRightFront, speedLeftBack);
        telemetry.update();
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    static public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    static public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
}
