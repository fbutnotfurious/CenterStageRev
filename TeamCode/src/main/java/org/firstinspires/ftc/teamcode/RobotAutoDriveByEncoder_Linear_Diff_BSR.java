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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

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

@Autonomous(name="Backstage Right Robot: Auto Drive By Encoder Diff", group="Robot")
//@Disabled
public class RobotAutoDriveByEncoder_Linear_Diff_BSR extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         leftDrive   = null;
    private DcMotor         rightDrive  = null;
    private DcMotor armLeft = null;
    private DcMotor armRight = null;
    private Servo gripper = null;
    private Servo wrist = null;
    private Servo launcher = null;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
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
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: HD Hes
    static final double     DRIVE_GEAR_REDUCTION    = 16.5;//20.15293 ;     // 4:1 and 5:1
    static final double     WHEEL_DIAMETER_INCHES   = 3.5433362205 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;//0.6
    static final double     TURN_SPEED              = 0.5;

    private final double gripperClosedPosition = 1.0;
    private final double gripperOpenPosition = 0.7;// 0.5
    private final double wristUpPosition = 1.5;//1.0 0.8
    private final double wristDownPosition = 0.4;// 0.5 0.4

    private final int armHomePosition = 0;
    private final int armIntakePosition = 10;
    private final int armScoreLeftPosition = -530;
    private final int armScoreRightPosition = 530;
    private final int armShutdownThreshold = 5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
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

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                leftDrive.getCurrentPosition(),
                rightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {

            //armLeft.setPower(0.3);
            //armRight.setPower(0.3);
            gripper.setPosition(gripperClosedPosition);
            wrist.setPosition(wristUpPosition);
            sleep(100);  // pause to display final telemetry message.

            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)

            // Drive to weigh point 1
            encoderDrive(DRIVE_SPEED, 17, 17, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            wrist.setPosition(wristDownPosition);
            sleep(100);  // pause to display final telemetry message.

            encoderDrive(DRIVE_SPEED, 12, 12, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            gripper.setPosition(gripperOpenPosition);
            sleep(100);  // pause to display final telemetry message.

            encoderDrive(DRIVE_SPEED, -3, -3, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            gripper.setPosition(gripperClosedPosition);
            sleep(400);  // pause to display final telemetry message.
            wrist.setPosition(wristUpPosition);
            encoderDrive(TURN_SPEED, 12, -12, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            encoderDrive(DRIVE_SPEED, -39, -39, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            //set arm,
            telemetry.addData("Arm Pos:",
                    "Actual left = " +
                            ((Integer) armLeft.getCurrentPosition()).toString() +
                            ", Actual right = " +
                            ((Integer) armRight.getCurrentPosition()).toString());
            telemetry.addData("Arm Pos:",
                    "left = " +
                            ((Integer) armLeft.getTargetPosition()).toString() +
                            ", right = " +
                            ((Integer) armRight.getTargetPosition()).toString());

            armLeft.setTargetPosition(armScoreLeftPosition);
            armRight.setTargetPosition(armScoreRightPosition);
            //armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armLeft.setPower(-0.3);
            armRight.setPower(0.3);


            while ((armLeft.isBusy() || armRight.isBusy())) {
                armLeft.setTargetPosition(armScoreLeftPosition);
                armRight.setTargetPosition(armScoreRightPosition);
                armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLeft.setPower(-0.3);
                armRight.setPower(0.3);
                // Display it for the driver.
                telemetry.addData("arm Running to", "left %7d and right %7d ",
                        armScoreLeftPosition,armScoreRightPosition);
                telemetry.addData("arm Left Currently at", " at %7d",
                        armLeft.getCurrentPosition());
                telemetry.addData("arm Right Currently at", " at %7d",
                        armRight.getCurrentPosition());
                telemetry.update();
            }

            gripper.setPosition(gripperOpenPosition);
            armLeft.setTargetPosition(0);
            armRight.setTargetPosition(0);
            //armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armLeft.setPower(0.3);
            armRight.setPower(-0.3);


            while ((armLeft.isBusy() || armRight.isBusy())) {
                armLeft.setTargetPosition(0);
                armRight.setTargetPosition(0);
                armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLeft.setPower(0.3);
                armRight.setPower(-0.3);
                // Display it for the driver.
                telemetry.addData("arm Running to", "left %7d and right %7d ",
                        armScoreLeftPosition,armScoreRightPosition);
                telemetry.addData("arm Left Currently at", " at %7d",
                        armLeft.getCurrentPosition());
                telemetry.addData("arm Right Currently at", " at %7d",
                        armRight.getCurrentPosition());
                telemetry.update();
            }

            // Park
            encoderDrive(DRIVE_SPEED, 12, 12, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            encoderDrive(TURN_SPEED, -9, 9, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            encoderDrive(DRIVE_SPEED, -25.5, -25.5, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout

            //RIGHT encoderDrive(TURN_SPEED,  5,  -5, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            //LEFT encoderDrive(TURN_SPEED,  -5,  5, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout


            //determine if Pixel on central spike
            // turn to right spike
            // determine if pixel on right spike
            // by now we know if pixel is on left, central or right
            // turn and drop off spike mark in position 1,2 or 3
            // Move arm down
            // Move wrist down
            // Open gripper

            if (PIXEL_RANDOMIZED == 1) {
                ;
            } else if (PIXEL_RANDOMIZED == 2) {
                ;
            } else {
                ;
            }
            // back up by 1 pixel
            // grip pixel
            //turn
            //drive to board
            // 3 point turn
            // Drop off by
            // moving arm
            // moving wrist
            // open gripper


            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);  // pause to display final telemetry message.
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
}
