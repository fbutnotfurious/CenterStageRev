/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Autonomous;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;
import static java.lang.Thread.sleep;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Autonomous Mechanum Robot: Auto Drive By Encoder", group="Robot")
@Disabled
public class RobotAutoDriveByEncoder_Linear extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    AprilTagDetection tagOfInterest = null;
    /* Declare OpMode members. */
    //private DcMotor         leftDrive   = null;
    //private DcMotor         rightDrive  = null;
    // declare and initialize four DcMotors.
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;
    private DcMotor motor_lift = null;
    private Servo gripperAsServo= null;


    private BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;
    int parkingobject=3;
    boolean initial_target=false;
    private double gripper =0;
    private ElapsedTime     runtime = new ElapsedTime();
    private ElapsedTime     runtimeObject = new ElapsedTime();
    private ElapsedTime     runtimeDetect = new ElapsedTime();
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7;//FIRST MEET //753.2;// Go Bilda Yellow 5203 1440 ;    // eg: TETRIX Motor Encoder
    //https://www.gobilda.com/content/spec_sheets/5203-2402-0019_spec_sheet.pdf
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // Miter gear
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953;//96 mm converted to inches(96/25.4), 1 inch is 25.4 mm;     // For figuring circumference
    //https://www.gobilda.com/96mm-mecanum-wheel-set-70a-durometer-bearing-supported-rollers/
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;// may have to change
    static final double     LIFT_SPEED            = 1.0;// 0.03may have to change

    static final double     TURN_SPEED              = 0.4;
    static final double     Lift_COUNTS_PER_INCH =100;//60



    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
  //  private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


  /*  private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };*/

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
  //  private static final String VUFORIA_KEY =
  //          "AUMvWrf/////AAABmbeDy0ZG+kA0qWc4y3DbUAYWHr4GbUWvLk218nrKBU9kU/84I5yQOIRnM2sBaHfEOcMg5mv41RlcEDlCq/hXkuTx5Sm3hHFgt4r6aXXJtT3OsHnDpCfQ/2Qxh32ctr5+K+qhXgQKLm7ewXcL1yNfy4hOg7ZzzelLOnNFWryKROrbgwPGyCDbsKmq0PtrFEB79By9XSEXHGt0UlaTYuCmqIqgYI0wWCoKkJwmpOGBpqc0nuFeN7Q2f1fB0cdwfaX3RTEiUJhLjmFfzxpOymsQQcHdsC1J7zNpWf5BkqZXxchFhkSYOM5JlVh0bHscr593OTLUsMUbZo8upiTqHgDYIHiEzAbTLI97y4piEEtqeaYJ";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    //private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    //private TFObjectDetector tfod;


    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        // Initialize the drive system variables.
        // leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        // rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        motor_lift   = hardwareMap.get(DcMotor.class, "motor_lift");
        gripperAsServo = hardwareMap.get(Servo.class, "gripperAsServo");


        //  initVuforia();
        //  initTfod();
        //if (tfod != null) {
        //    tfod.activate();

        // The TensorFlow software will scale the input images from the camera to a lower resolution.
        // This can result in lower detection accuracy at longer distances (> 55cm or 22").
        // If your target is at distance greater than 50 cm (20") you can increase the magnification value
        // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
        // should be set to the value of the images used to create the TensorFlow Object Detection model
        // (typically 16/9).
        // tfod.setZoom(1.4, 16.0/9.0);
          /*  tfod.setZoom(1.5, 16.0/9.0);
        }*/


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        /*leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);*/

        //leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // Retrieve the IMU from the hardware map
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!imu.isGyroCalibrated()) {
            try {
                sleep(50);
                throw new InterruptedException("Exception:Checking if gyro is calibrated.");
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            //idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d:%7d:%7d",
                front_left.getCurrentPosition(),
                front_right.getCurrentPosition(),
                back_left.getCurrentPosition(),
                back_right.getCurrentPosition());
        telemetry.update();
        motor_lift.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start
        runtimeDetect.reset();
        while (runtimeDetect.seconds()<=5)
        {
;
    }
        waitForStart();
            runtimeObject.reset();
            while (opModeIsActive()) {
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                if (currentDetections.size() != 0) {
                    boolean tagFound = false;

                    for (AprilTagDetection tag : currentDetections) {
                        if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                            tagOfInterest = tag;
                            tagFound = true;
                            break;
                        }
                    }

                    if (tagFound) {
                        telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                        tagToTelemetry(tagOfInterest);
                    } else {
                        telemetry.addLine("Don't see tag of interest :(");

                        if (tagOfInterest == null) {
                            telemetry.addLine("(The tag has never been seen)");
                        } else {
                            telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                            tagToTelemetry(tagOfInterest);
                        }
                    }

                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }

                }

                telemetry.update();
                if (tagOfInterest != null) {
                    telemetry.addLine("Tag snapshot:\n");
                    tagToTelemetry(tagOfInterest);
                    telemetry.update();
                } else {
                    telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                    telemetry.update();
                }

                /* Actually do something useful */
                if (tagOfInterest == null) {
                    /*
                     * Insert your autonomous code here, presumably running some default configuration
                     * since the tag was never sighted during INIT
                     */
                } else {
                    /*
                     * Insert your autonomous code here, probably using the tag pose to decide your configuration.
                     */

                    // e.g.
                    if (tagOfInterest.id == LEFT) {
                        telemetry.addData("parkingobject", "1");
                        parkingobject = 1;
                    } else if (tagOfInterest.id == MIDDLE) {
                        telemetry.addData("parkingobject", "2");
                        parkingobject = 2;
                    } else if (tagOfInterest.id == RIGHT) {
                        telemetry.addData("parkingobject", "3");
                        parkingobject = 3;
                        // do something else
                    }
                }
                gripper = 0.23;
                //gripperAsServo.getController().pwmEnable();
                telemetry.addData("Setting gripper position %f", gripper);
                if (gripperAsServo.getPosition() != gripper) {
                    gripperAsServo.setPosition(gripper);
                }
                sleep(1000);
                //encoderLift(LIFT_SPEED,27,3.0);//0.1

                encoderDrive(DRIVE_SPEED, 33, 33, 5.0);
                //encoderDrive(DRIVE_SPEED, -8, -8, 5.0);
                //encoderDrive(TURN_SPEED/4, 12.0, -12.0, 4.0);
                //encoderDrive(DRIVE_SPEED, 6.5, 6.5, 5.0);
                sleep(1000);

                gripper = 0.05;
                //gripperAsServo.getController().pwmEnable();
                telemetry.addData("Setting gripper position %f", gripper);
                if (gripperAsServo.getPosition() != gripper)
                {
                    gripperAsServo.setPosition(gripper);
                }
                sleep(1000);
                //encoderDrive(DRIVE_SPEED, -6.5, -6.5, 5.0);
                //encoderDrive(TURN_SPEED/4, -12, 12, 4.0);

                //(DRIVE_SPEED, 21.5, 21.5, 5.0);
                //13.55, -8,-12
                // removed code


                if (parkingobject == 1) {
                    telemetry.addData("parkingobject", "1");
                    encoderDrive(TURN_SPEED, -20, 20, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
                    encoderDrive(DRIVE_SPEED, 22, 22, 5.0);
                    break;
                } else if (parkingobject == 2) {
                    telemetry.addData("parkingobject", "2");
                    //encoderDrive(DRIVE_SPEED, -1, -1, 5.0);

                    break;
                } else if (parkingobject == 3) {
                    telemetry.addData("parkingobject", "3");
                    //encoderDrive(TURN_SPEED, 21, -21, 4.0);
                    //encoderDrive(DRIVE_SPEED, 21.0, 21.0, 5.0);
                    //encoderDrive(TURN_SPEED, 20, -20, 4.0);
                    //encoderDrive(DRIVE_SPEED, 22, 22, 5.0);
                    break;// S2: Turn Right 12 Inches with 4 Sec timeout
                    //encoderDrive(DRIVE_SPEED, 27, 24, 5.0);

                }
               //telemetry.update();

                // Step through each leg of the path,


                // }


                // Note: Reverse movement is obtained by setting a negative distance (not speed)
                //encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
                //encoderDrive(DRIVE_SPEED,  -40,  -48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout

                //encoderDrive(TURN_SPEED,   18, -18, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
            }    //encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
                double botHeading = -getAngle();
                String filename = "SavedHeadings.json";
                String headingData = "Heading=" + botHeading;
                File file = AppUtil.getInstance().getSettingsFile(filename);
                ReadWriteFile.writeFile(file, headingData);
                String readData = ReadWriteFile.readFile(file);
                double readbotHeading = 0;
                readbotHeading = Double.parseDouble(readData.substring(8));
                sleep(1000);  // pause to display final telemetry message.
                telemetry.addData("HeadingReadfromFile is", "%.3f", readbotHeading);


            //telemetry.addData("savedfile", t);
            telemetry.update();

    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newfront_LeftTarget;
        int newback_LeftTarget;
        int newfront_RightTarget;
        int newback_RightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfront_LeftTarget = front_left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newback_LeftTarget = back_left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);

            newfront_RightTarget = front_right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newback_RightTarget = back_right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            front_left.setTargetPosition(newfront_LeftTarget);
            back_left.setTargetPosition(newback_LeftTarget);

            front_right.setTargetPosition(newfront_RightTarget);
            back_right.setTargetPosition(newfront_RightTarget);

            // Turn On RUN_TO_POSITION
            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            front_left.setPower(Math.abs(speed));
            back_left.setPower(Math.abs(speed));
            front_right.setPower(Math.abs(speed));
            back_right.setPower(Math.abs(speed));
            telemetry.addData("o gripper position %f", gripperAsServo.getPosition());


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (front_left.isBusy() && back_left.isBusy()
                    && front_right.isBusy() && back_right.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newfront_LeftTarget,
                        newback_LeftTarget, newfront_RightTarget, newback_RightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                                            front_left.getCurrentPosition(), back_left.getCurrentPosition(),
                                            front_right.getCurrentPosition(), back_right.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            front_left.setPower(0);
            back_left.setPower(0);
            front_right.setPower(0);
            back_right.setPower(0);

            // Turn off RUN_TO_POSITION
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -3.14)
            deltaAngle += 6.28;
        else if (deltaAngle > 3.14)
            deltaAngle -= 6.28;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
     //   VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

       // parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        //vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    /*private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }*/
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        //telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        //telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        //telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public void encoderLift(double speed,
                            double liftInches,
                            double timeoutS) {

        int newmotor_lift_Target;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newmotor_lift_Target = motor_lift.getCurrentPosition() + (int) (liftInches * Lift_COUNTS_PER_INCH);
        motor_lift.setTargetPosition(newmotor_lift_Target);

        // Turn On RUN_TO_POSITION
        motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        motor_lift.setPower(Math.abs(speed));


        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        //while ((runtime.seconds() < timeoutS) &&

        while ((runtime.seconds() < timeoutS) &&
                (motor_lift.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Lift Running to", " %7d", newmotor_lift_Target);
            telemetry.addData("Lift Currently at", " at %7d",
                    motor_lift.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;

        motor_lift.setPower(0.005);//0.01

        // Turn off RUN_TO_POSITION
       // motor_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
