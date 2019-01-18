package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.google.gson.annotations.Expose;
import com.google.gson.annotations.SerializedName;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.List;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Corner Aut", group="Pushbot")
public class LinearAutonomousTwo extends LinearOpMode
{
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AV5J3VH/////AAABmUzrZLjpXUnZhQkg7Cftp+ocILx7axVLhz/mYK9YOh+/wkUzBIN5ItpeP7EECqQ/doezbGxGOZ3jdbqdSVQhNyd4lYs6HEgOhENyUUxd44Zsa7Y8TWMep9sb4jBUH2rxowGjJfEwIHFD5FFgeHvGFo8Cc2F0gBTs9sN8p5QTo/f3a7mD0rGqJiCozJvY9xnJRYDz3uiR+Mzk9dc/4YWD/m5NYzUTIHAuieSloUYWLatwMrREoz04312bEWIALaKMCmHdmvrWxJTZeohHLUdeyBj2E0L2MQD/xbUyM9HBMdFGY6wel/dko7X6kuTV1mTdU89qj9d4T0jZVH3+l2sSpR2uApcZZ0QbJTXMdQFL1Tfb";
    VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double SLOW_SPEED = 0.3;
    static final double TURN_SPEED = 0.5;

    private DcMotor leftMotorFront = null;
    private DcMotor rightMotorFront = null;
    private DcMotor leftMotorBack = null;
    private DcMotor rightMotorBack = null;
    private DcMotor actuatorMotor = null;
    private DcMotor horizontalRailMotor = null;
    private DcMotor verticalRailMotor = null;
    private DcMotor collectionMotor = null;

    private Servo beaconServo = null;
    private Servo rackPinionServo = null;

    private WebcamName webcam = null;
    private ColorSensor colorSensor = null;

    @Override
    public void runOpMode()
    {
        int goldLocation = -1;
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        leftMotorFront = hardwareMap.get(DcMotor.class, "left_motor_front");
        rightMotorFront = hardwareMap.get(DcMotor.class, "right_motor_front");
        leftMotorBack = hardwareMap.get(DcMotor.class, "left_motor_back");
        rightMotorBack = hardwareMap.get(DcMotor.class, "right_motor_back");
        actuatorMotor = hardwareMap.get(DcMotor.class, "actuator_motor");
        horizontalRailMotor = hardwareMap.get(DcMotor.class, "horizontal_rail_motor");
        verticalRailMotor = hardwareMap.get(DcMotor.class, "vertical_rail_motor");
        collectionMotor = hardwareMap.get(DcMotor.class, "collection_motor");

        beaconServo = hardwareMap.get(Servo.class, "beacon_servo");

        //webcam = hardwareMap.get(WebcamName.class, "MainCam");
        //colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        rightMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        /*leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                leftMotorFront.getCurrentPosition(),
                rightMotorFront.getCurrentPosition(),
                leftMotorBack.getCurrentPosition(),
                rightMotorBack.getCurrentPosition());
        telemetry.update();

        beaconServo.setPosition(Servo.MIN_POSITION);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    goldLocation = 1;
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    goldLocation = 2;
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    goldLocation = 3;
                                }
                            }
                        }
                        telemetry.update();
                    }
                    telemetry.addData("Status", "Started");
                    telemetry.addData("Debug", "Gold location Var = " + goldLocation);
                    telemetry.update();
                    ToggleActuator(true);
                    while(actuatorMotor.isBusy() && opModeIsActive())
                    {
                        //Do Nothing
                    }
                    if(goldLocation == 1)
                    {
                        encoderDrive(1.0, 10, -10, -10, 10, .1);
                    } else if(goldLocation == 2)
                    {
                        encoderDrive(1.0, -5, -5 , -5, -5, .1);
                        encoderDrive(1.0, -10, 10, 10, -10, .1);
                    } else if(goldLocation == 3)
                    {

                    }
                }
            }
        }

        if (tfod != null) {
            //tfod.shutdown();
        }


        ///Move to corner
        ///Place beacon
        ///Park in crater

//Region Timing
        /*ToggleActuator(false); //Old (Not Encoders)
        moveTime(1.0, -1.0, -1.0, 1.0, .5);
        moveTime(-1.0, -1.0, -1.0, -1.0, 1.15);
        sleep(1000);
        moveTime(-1.0, -1.0, 1.0, 1.0, 1.1);
        Deposit();
        sleep(800);
        moveTime(-1.0, -1.0, 1.0, 1.0, .45);
        sleep(800);
        moveTime(-1.0, -1.0, -1.0, -1.0, 1.85);
//End Region
*/
        //Encoders





        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             int leftTicksFront, int rightTicksFront,
                             int leftTicksBack,  int rightTicksBack,
                             double timeoutS)
    {
        /*int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotorFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotorFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftMotorFront.setTargetPosition(newLeftTarget);
            rightMotorFront.setTargetPosition(newRightTarget);
            leftMotorBack.setTargetPosition(newLeftTarget);
            rightMotorBack.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotorFront.setPower(Math.abs(speed));
            rightMotorFront.setPower(Math.abs(speed));
            leftMotorBack.setPower(Math.abs(speed));
            rightMotorBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotorFront.isBusy() && rightMotorFront.isBusy()))
            {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        leftMotorFront.getCurrentPosition(),
                        rightMotorFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            rightMotorFront.setPower(0);
            leftMotorFront.setPower(0);
            rightMotorBack.setPower(0);
            leftMotorBack.setPower(0);


            // Turn off RUN_TO_POSITION
            leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move
        }*/
        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotorFront.setTargetPosition(leftTicksFront);
        leftMotorBack.setTargetPosition(leftTicksBack);
        rightMotorFront.setTargetPosition(rightTicksFront);
        rightMotorBack.setTargetPosition(rightTicksBack);

        leftMotorFront.setPower(1.0);
        leftMotorBack.setPower(1.0);
        rightMotorFront.setPower(1.0);
        rightMotorBack.setPower(1.0);
        while(leftMotorFront.isBusy() && opModeIsActive() || rightMotorFront.isBusy() && opModeIsActive())
        {
            //Wait
        }
        leftMotorFront.setPower(0);
        leftMotorBack.setPower(0);
        rightMotorFront.setPower(0);
        rightMotorBack.setPower(0);

    }

    boolean toggled = false;

    public void ToggleActuator(boolean encoderDrive)
    {
        if(!encoderDrive)
        {
            ElapsedTime rt = new ElapsedTime();
            rt.reset();
            actuatorMotor.setPower(1.0);
            //runtime.reset();
            while (opModeIsActive() && (rt.seconds() < 8.35))
            {
                telemetry.addData("Path", "Leg 1: %2.5f", rt.seconds());
                telemetry.update();

            }
            actuatorMotor.setPower(0.0);
            sleep(100);
        } else if(encoderDrive)
        {
            actuatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            actuatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            actuatorMotor.setTargetPosition(3300);
            actuatorMotor.setPower(1.0);
            while(actuatorMotor.isBusy() && opModeIsActive())
            {
                telemetry.addData("Status", "Actuator Busy");
            }
            actuatorMotor.setPower(0);
        } else {
            telemetry.addData("Error", "Bool encoderDrive is set to null");
        }

    }

    //lbrf
    public void moveTime(double leftFrontSpeed, double leftBackSpeed, double rightFrontSpeed, double rightBackSpeed, double duration)
    {
        ElapsedTime rt = new ElapsedTime();
        rt.reset();
        leftMotorFront.setPower(leftFrontSpeed);
        rightMotorFront.setPower(rightFrontSpeed); //To go straight, right speed needs to be positve
        leftMotorBack.setPower(leftBackSpeed);
        rightMotorBack.setPower(rightBackSpeed);
        while (opModeIsActive() && (rt.seconds() < duration))
        {
            telemetry.addData("Path", "temp");
            telemetry.update();
        }
        leftMotorFront.setPower(0.0);
        rightMotorFront.setPower(0.0);
        leftMotorBack.setPower(0.0);
        rightMotorBack.setPower(0.0);
        sleep(100);
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void Deposit()
    {
        double start = Servo.MIN_POSITION;
        double end = Servo.MAX_POSITION;

        beaconServo.setPosition(start);

        while (beaconServo.getPosition() < end)
        {
            beaconServo.setPosition(beaconServo.getPosition() + .05);
        }
    }

    public void CheckForColor()
    {
        float[] colors = GetSensorColor();

        if(colors[4] <= 44)
        {
            moveTime(1.0, -1.0, -1.0, 1.0, .5);
            moveTime(-1.0, -1.0, -1.0, -1.0, .25);
        } else if(colors[4] >= 45)
        {
            sleep(200);
            moveTime(-1.0, 1.0, 1.0, -1.0, .8);
            moveTime(-1.0, -1.0, -1.0, -1.0, .2);
            float[] colors2 = GetSensorColor();
            if(colors2[4] <= 44)
            {
                sleep(200);
                moveTime(1.0, -1.0, -1.0, 1.0, .5);
                moveTime(-1.0, -1.0, -1.0, -1.0, .25);
            } else if(colors2[4] >= 45)
            {
                sleep(200);
                moveTime(-1.0, 1.0, 1.0, -1.0, .8);
                //moveTime(1.0, -1.0, -1.0, 1.0, .5);
                moveTime(-1.0, -1.0, -1.0, -1.0, .25);
            }
        } else {
            telemetry.addData("Error", "Ball or Cube not found");
        }
    }

    public float[] GetSensorColor()
    {
        float[] HSV = {0F, 0F, 0F};
        Color.RGBToHSV((colorSensor.red() * 255),
                (colorSensor.green() * 255),
                (colorSensor.blue() * 255),
                HSV);
        return new float[]  {/*Red*/  colorSensor.red(),
                /*Green*/colorSensor.green(),
                /*Blue*/ colorSensor.blue(),
                /*Alpha*/colorSensor.alpha(),
                /*Hue*/  HSV[0]};
    }

    /*public static void deserialize() {
        Command c = new Command();

        ObjectMapper mapper = new ObjectMapper();

        File file = new File("artist.json");
        try {
            // Serialize Java object info JSON file.
            mapper.writeValue(file, artist);
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            // Deserialize JSON file into Java object.
            Command c = mapper.readValue(file, Command.class);
            System.out.println("newArtist.getId() = " + newArtist.getId());
            System.out.println("newArtist.getName() = " + newArtist.getName());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }*/
}
