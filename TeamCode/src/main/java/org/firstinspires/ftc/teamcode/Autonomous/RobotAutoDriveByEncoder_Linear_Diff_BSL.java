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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.Pipelines.Prop;
import org.firstinspires.ftc.teamcode.Pipelines.StartPosition;
import org.firstinspires.ftc.teamcode.Pipelines.WebcamPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.PathConstants;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
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
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Backstage Blue Robot: Auto Drive By Encoder Diff", group="Robot")
//@Disabled
public class RobotAutoDriveByEncoder_Linear_Diff_BSL extends LinearOpMode {

    OpenCvCamera webcam;
    /* Declare OpMode members. */
    private DcMotor         leftDrive   = null;
    private DcMotor         rightDrive  = null;
    private DcMotor armLeft = null;
    private DcMotor armRight = null;
    private Servo gripper = null;
    private Servo wrist = null;
    private Servo launcher = null;

    OpenCvCamera camera;
    //AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // MUST VERIFY
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    int PIXEL_RANDOMIZED=3;
    AprilTagDetection tagOfInterest = null;

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    //REV-41-1291
    //https://docs.revrobotics.com/ultraplanetary/

    // Robot specifications
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: HD Hes
    static final double     DRIVE_GEAR_REDUCTION    = 16.5;//20.15293 ;     // 4:1 and 5:1
    static final double     WHEEL_DIAMETER_INCHES   = 3.5433362205 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    // Calibration details
    static final double     DRIVE_SPEED             = 0.3;//0.6
    static final double     TURN_SPEED              = 0.5;

    private final double gripperClosedPosition = 1.0;
    private final double gripperOpenPosition = 0.7;// 0.5
    private final double wristUpPosition = 1.5;//1.0 0.8
    private final double wristDownPosition = 0.4;// 0.5 0.4

    private final int armHomePosition = 0;
    private final int armIntakePosition = 200;
    private final int armScoreLeftPosition = -530;
    private final double armSpeed=0.3;
    //private final int armScoreRightPosition = -armScoreLeftPosition;// always
    private final int armShutdownThreshold = 5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables. Perform the right hardware mapping
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        armRight = hardwareMap.get(DcMotor.class, "armRight");
        gripper = hardwareMap.get(Servo.class, "gripper");
        wrist = hardwareMap.get(Servo.class, "wrist");
        launcher = hardwareMap.get(Servo.class, "launcher");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Enabling RUN_USING_ENCODER for the drive and arm
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                leftDrive.getCurrentPosition(),
                rightDrive.getCurrentPosition());
        telemetry.update();

        // Initialize gripper and wrist
        gripper.setPosition(gripperClosedPosition);
        wrist.setPosition(wristUpPosition);
        sleep(100);  // pause to display final telemetry message.

        // Camera integration
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        WebcamPipeline detector = new WebcamPipeline(telemetry, StartPosition.BLUE_STAGE);
        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        int totalTimeWaited = 0;
        boolean pipelineRan = true;
        if(detector.getPropLocation() == null) {
            telemetry.addData("ERROR", "Start was pressed too soon.");
            telemetry.update();

            while(detector.getPropLocation() == null && totalTimeWaited < 7000) {
                totalTimeWaited += (webcam.getOverheadTimeMs() * 4);
                sleep(webcam.getOverheadTimeMs() * 4L);
            }
            telemetry.addData("Wasted time", totalTimeWaited);
            if(totalTimeWaited > 7000) {
                telemetry.addData("ERROR", "The pipeline never ran.");
                pipelineRan = false;
            }
            telemetry.update();
        }
        else {
            telemetry.addData("INFO", "Pipeline is running correctly");
            telemetry.update();
        }
        Prop location = Prop.CENTER;

        if(pipelineRan) {
            location = detector.getPropLocation();
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }

        telemetry.addData("Running path", " BLUE_STAGE_" + location);
        telemetry.update();
        while (opModeIsActive())
        {
            if (location==Prop.CENTER) {
                // Note: Reverse movement is obtained by setting a negative distance (not speed)

                // * Approaching the team prop *
                //S1: Drive forward for BSL_ForwardPoint1
                encoderDrive(DRIVE_SPEED, PathConstants.BSL_FowardPoint1, PathConstants.BSL_FowardPoint1, 5.0);
                //S2: Set the wrist to down position
                wrist.setPosition(wristDownPosition);
                sleep(100);  // pause to display final telemetry message.
                //S3: Drive forward for BSL_FowardPoint2
                encoderDrive(DRIVE_SPEED, PathConstants.BSL_FowardPoint2, PathConstants.BSL_FowardPoint2, 5.0);

                //* Drop the purple pixel *
                //S4: Open the gripper to drop the purple pixel
                runtime.reset();
                if (gripper.getPosition() != gripperOpenPosition) {
                    gripper.setPosition(gripperOpenPosition);// Open Gripper to drop of pixel
                }
                sleep(100);  // pause to display final telemetry message.

                //S5: Travel backward to prepare for yellow pixel pickup
                encoderDrive(DRIVE_SPEED, PathConstants.BSL_BackwardPoint1, PathConstants.BSL_BackwardPoint1, 5.0);
                sleep(100);  // pause to display final telemetry message.
                //* Pick up the yellow pixel *
                //S6: Close the gripper to pick up the yellow pixel
                runtime.reset();
                if (gripper.getPosition() != gripperClosedPosition) {
                    gripper.setPosition(gripperClosedPosition);// Open Gripper to drop of pixel
                }
                sleep(100);  // pause to display final telemetry message.
                //S7: Move arm up to perpare for travel
                wrist.setPosition(wristUpPosition);
                sleep(100);  // pause to display final telemetry message.
                //S8: Drive backward for 1in to ensure the final drop will happen at the center tag
                encoderDrive(DRIVE_SPEED, -1, -1, 5.0);

                //sleep(100);  // pause to display final telemetry message.
                //S9: Turn right to face against the backdrop board
                encoderDrive(TURN_SPEED, PathConstants.BSL_TurnLeft1-1  , -PathConstants.BSL_TurnLeft1+1, 5.0);

                //* Brackdrop board execution
                //S10: Drive backward for BSL_BackwardPoint2 to approach the board
                encoderDrive(DRIVE_SPEED, PathConstants.BSL_BackwardPoint2, PathConstants.BSL_BackwardPoint2, 5.0);
                //S11: Move arm to the score position
                encoderArm(armSpeed*2, armScoreLeftPosition, 5.0);

                //S12: Open the gripper to drop the yellow pixel
                runtime.reset();
                do {
                    gripper.setPosition(gripperOpenPosition);
                } while (gripper.getPosition() != gripperOpenPosition | (runtime.seconds() < 1.0));
                sleep(100);


                //* Parking to destination *
                //S13: Drive forward 12 in
                encoderDrive(DRIVE_SPEED, 12, 12, 5.0);
                //S14: Move are to IntakePosition
                encoderArm(-armSpeed*2, armIntakePosition, 5.0);
                //S15: Turn left 9in
                encoderDrive(TURN_SPEED, 9, -9, 5.0);
                //S16: Drive backward 19.5 in to the final parking postion
                encoderDrive(DRIVE_SPEED*2, -19.5, -19.5, 5.0);  
                telemetry.addData("Path", "Complete");
                telemetry.update();
                sleep(100);  // pause to display final telemetry message.*/
            }
            else if (location==Prop.LEFT)
            {
                // Step through each leg of the path,
                // Note: Reverse movement is obtained by setting a negative distance (not speed)
                ;

                // Drive to weigh point 1
                encoderDrive(DRIVE_SPEED, PathConstants.BSL_FowardPoint1, PathConstants.BSL_FowardPoint1, 5.0);  // S1: Forward 17 Inches with 5 Sec timeout
                wrist.setPosition(wristDownPosition);
                sleep(100);  // pause to display final telemetry message.
                encoderDrive(DRIVE_SPEED, PathConstants.BSL_FowardPoint2+3, PathConstants.BSL_FowardPoint2+3, 5.0);  // S1: Forward 12 Inches with 5 Sec timeout

                encoderDrive(TURN_SPEED, PathConstants.BSL_TurnLeft1+1  , -PathConstants.BSL_TurnLeft1+1, 5.0);  // S1: Forward 12 Inches with 5 Sec timeout
                encoderDrive(DRIVE_SPEED, -21, -21, 5.0);  // S1: Forward -3 Inches with 5 Sec timeout


                runtime.reset();
                if (gripper.getPosition() != gripperOpenPosition) {
                    gripper.setPosition(gripperOpenPosition);// Open Gripper to drop of pixel
                }
                sleep(100);  // pause to display final telemetry message.


                encoderDrive(DRIVE_SPEED, PathConstants.BSL_BackwardPoint1, PathConstants.BSL_BackwardPoint1, 5.0);  // S1: Forward -3 Inches with 5 Sec timeout
                sleep(100);  // pause to display final telemetry message.

                runtime.reset();
                if (gripper.getPosition() != gripperClosedPosition) {
                    gripper.setPosition(gripperClosedPosition);// Open Gripper to drop of pixel
                }
                sleep(100);  // pause to display final telemetry message.
                wrist.setPosition(wristUpPosition);
                sleep(100);  // pause to display final telemetry message.

                encoderDrive(DRIVE_SPEED, PathConstants.BSL_BackwardPoint2+25,
                        PathConstants.BSL_BackwardPoint2+25, 5.0);  // S1: Forward -38 Inches with 5 Sec timeout
                encoderArm(armSpeed*2, armScoreLeftPosition, 5.0);
                runtime.reset();
                do {
                    gripper.setPosition(gripperOpenPosition);// Open Gripper to drop of pixel
                } while (gripper.getPosition() != gripperOpenPosition | (runtime.seconds() < 1.0));
                sleep(100);


                // Park
                encoderDrive(DRIVE_SPEED, 12, 12, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout

                encoderArm(-armSpeed*2, armIntakePosition, 5.0);
                encoderDrive(TURN_SPEED, 7, -7, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
                encoderDrive(DRIVE_SPEED*2, -22, -22, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
                telemetry.addData("Path", "Complete");
                telemetry.update();
                sleep(100);  // pause to display final telemetry message.*/
            }
            else
            {
                // Step through each leg of the path,
                // Note: Reverse movement is obtained by setting a negative distance (not speed)
                ;

                // Drive to weigh point 1
                encoderDrive(DRIVE_SPEED, PathConstants.BSL_FowardPoint1, PathConstants.BSL_FowardPoint1, 5.0);  // S1: Forward 17 Inches with 5 Sec timeout
                wrist.setPosition(wristDownPosition);
                sleep(100);  // pause to display final telemetry message.
                encoderDrive(DRIVE_SPEED, PathConstants.BSL_FowardPoint2+2, PathConstants.BSL_FowardPoint2+2, 5.0);  // S1: Forward 12 Inches with 5 Sec timeout

                encoderDrive(TURN_SPEED, +PathConstants.BSL_TurnLeft1  , -PathConstants.BSL_TurnLeft1, 5.0);  // S1: Forward 12 Inches with 5 Sec timeout
                encoderDrive(DRIVE_SPEED, 3.5, 3.5, 5.0);  // S1: Forward 12 Inches with 5 Sec timeout


                runtime.reset();
                if (gripper.getPosition() != gripperOpenPosition) {
                    gripper.setPosition(gripperOpenPosition);// Open Gripper to drop of pixel
                }
                sleep(100);  // pause to display final telemetry message.


                encoderDrive(DRIVE_SPEED, PathConstants.BSL_BackwardPoint1, PathConstants.BSL_BackwardPoint1, 5.0);  // S1: Forward -3 Inches with 5 Sec timeout
                runtime.reset();
                sleep(100);  // pause to display final telemetry message.

                if (gripper.getPosition() != gripperClosedPosition) {
                    gripper.setPosition(gripperClosedPosition);// Open Gripper to drop of pixel
                }
                sleep(100);  // pause to display final telemetry message.
                wrist.setPosition(wristUpPosition);
                sleep(100);  // pause to display final telemetry message.
                encoderDrive(TURN_SPEED, -2 , 2, 5.0);  // S1: Forward 12 Inches with 5 Sec timeout

                encoderDrive(DRIVE_SPEED, PathConstants.BSL_BackwardPoint2, PathConstants.BSL_BackwardPoint2, 5.0);  // S1: Forward -38 Inches with 5 Sec timeout
                encoderArm(armSpeed*2, armScoreLeftPosition, 5.0);
                runtime.reset();
                do {
                    gripper.setPosition(gripperOpenPosition);// Open Gripper to drop of pixel
                } while (gripper.getPosition() != gripperOpenPosition | (runtime.seconds() < 1.0));
                sleep(100);
                // Park
                encoderDrive(DRIVE_SPEED, 12, 12, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
                //encoderArm(armSpeed, armIntakePosition,8.0);
                encoderArm(-armSpeed*2, armIntakePosition, 5.0);

                encoderDrive(TURN_SPEED, 9, -9, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
                encoderDrive(DRIVE_SPEED*2, -31.5, -31.5, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
                telemetry.addData("Path", "Complete");
                telemetry.update();
                sleep(100);  // pause to display final telemetry message.*/
            }
            terminateOpModeNow();


        }
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                                            leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    public void encoderArm( double speed,int EncoderTarget,
                            double timeoutS) {
            runtime.reset();
            armLeft.setTargetPosition(EncoderTarget);
            armRight.setTargetPosition(-EncoderTarget);
            armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armLeft.setPower(-speed);
            armRight.setPower(speed);


            while ((armLeft.isBusy() || armRight.isBusy()) &&(runtime.seconds() < timeoutS)) {
                armLeft.setTargetPosition(EncoderTarget);
                armRight.setTargetPosition(-EncoderTarget);
                armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLeft.setPower(-speed);
                armRight.setPower(speed);
                // Display it for the driver.
               telemetry.addData("arm Running to", "left %7d and right %7d ",
                        EncoderTarget,-EncoderTarget);
                telemetry.addData("arm Left Currently at", " at %7d",
                        armLeft.getCurrentPosition());
                telemetry.addData("arm Right Currently at", " at %7d",
                        armRight.getCurrentPosition());
                telemetry.update();
            }

    }
}
