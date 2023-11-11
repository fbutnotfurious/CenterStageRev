package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Starter Bot 2024 Final control", group="Iterative Opmode")

public class StarterBot2024TeleopChangesmapping extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armLeft = null;
    private DcMotor armRight = null;
    private Servo gripper = null;
    private Servo wrist = null;
    private Servo launcher = null;

    private boolean manualMode = false;
    private double armSetpoint = 0.0;

    //private final double armManualDeadband = 0.03;
    private final double armManualDeadband = 0.15;

    private final double gripperClosedPosition = 1.0;
    private final double gripperOpenPosition = 0.7;// 0.5
    private final double wristUpPosition = 1.5;//1.0 0.8
    private final double wristDownPosition = 0.4;// 0.5 0.4

    private final int armHomePosition = 0;
    private final int armIntakePosition = 10;
    private final int armScorePosition = 300;
    private final int armShutdownThreshold = 5;

    private final double launcherInitial=0.0;
    private final double launcherFinal=0.8;
    private boolean hangingStatus=false;
    // constant for slow speed
    private final double driveMotorSlowSpeed = 0.55;
    //constant for fast speed
    private final double driveMotorFastSpeed = 0.75;
     /* Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        armLeft  = hardwareMap.get(DcMotor.class, "armLeft");
        armRight = hardwareMap.get(DcMotor.class, "armRight");
        gripper = hardwareMap.get(Servo.class, "gripper");
        wrist = hardwareMap.get(Servo.class, "wrist");
        launcher = hardwareMap.get(Servo.class, "launcher");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        armLeft.setDirection(DcMotor.Direction.FORWARD);
        armRight.setDirection(DcMotor.Direction.REVERSE);
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // armLeft.setPower(0.0);
       // armRight.setPower(0.0);

        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setTargetPosition(armHomePosition);
        armRight.setTargetPosition(armHomePosition);
        armLeft.setPower(1.0);
        armRight.setPower(1.0);
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLeft.setPower(0);
        armRight.setPower(0);

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double leftPower;
        double rightPower;
        double manualArmPower;

        //DRIVE
        double drive = Math.pow(gamepad2.left_stick_y,3);
        double turn  =  Math.pow(gamepad2.right_stick_x,3);
        if (gamepad2.right_bumper)
        {
            launcher.setPosition(launcherFinal);

        }
        else if(gamepad2.left_bumper)
        {
            launcher.setPosition(launcherInitial);

        }
       // leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
       // rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        leftPower    = Range.clip(drive + turn, -driveMotorSlowSpeed, driveMotorSlowSpeed) ;//0.55
        rightPower   = Range.clip(drive - turn, -driveMotorSlowSpeed, driveMotorSlowSpeed) ;//0.55

        if (gamepad2.left_trigger>0)
        {
            leftPower    = Range.clip(drive + turn, -driveMotorFastSpeed, driveMotorFastSpeed) ;//0.55
            rightPower   = Range.clip(drive - turn, -driveMotorFastSpeed, driveMotorFastSpeed) ;//0.55

        }

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        //ARM & WRIST
        manualArmPower = Math.pow(gamepad1.right_trigger,3) - Math.pow(gamepad1.left_trigger,3);

        if (gamepad2.y==true) {
            hangingStatus = true;
        }
        if (gamepad2.a==true) {
            hangingStatus = false;
        }

        if (hangingStatus==true)
        {
            telemetry.addData(" it is in hanging mode",manualArmPower);
            armLeft.setPower(0.5);
            armRight.setPower(0.5);
        }

        if (Math.abs(manualArmPower) > armManualDeadband ) {
            if (!manualMode) {
                armLeft.setPower(0.0);
                armRight.setPower(0.0);
                armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                manualMode = true;
            }
            armLeft.setPower(manualArmPower);
            armRight.setPower(manualArmPower);
        }
        else {
            if (manualMode) {
                armLeft.setTargetPosition(armLeft.getCurrentPosition());
                armRight.setTargetPosition(armRight.getCurrentPosition());
                armLeft.setPower(0.3);
                armRight.setPower(0.3);
                armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                manualMode = false;
            }

            //preset buttons
          if (gamepad1.a) {
                wrist.setPosition(wristDownPosition);
            }
        else if (gamepad1.y) {
                wrist.setPosition(wristUpPosition);
            }
        }

        //Re-zero encoder button
        if (gamepad1.start) {
            armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armLeft.setPower(0.0);
            armRight.setPower(0.0);
            manualMode = false;
        }

        //Watchdog to shut down motor once the arm reaches the home position
        if (!manualMode &&
                armLeft.getMode() == DcMotor.RunMode.RUN_TO_POSITION &&
                armLeft.getTargetPosition() <= armShutdownThreshold &&
                armLeft.getCurrentPosition() <= armShutdownThreshold
        ) {
            armLeft.setPower(0.0);
            armRight.setPower(0.0);
            armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        //GRIPPER
        if (gamepad1.b) {
            gripper.setPosition(gripperOpenPosition);
        }
        else if (gamepad1.x) {
            gripper.setPosition(gripperClosedPosition);
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Gamepad", "drive (%.2f), turn (%.2f)", drive, turn);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Manual Power", manualArmPower);
        telemetry.addData("Arm Pos:",
                "Actual left = " +
                        ((Integer)armLeft.getCurrentPosition()).toString() +
                        ", Actual right = " +
                        ((Integer)armRight.getCurrentPosition()).toString());
        telemetry.addData("Arm Pos:",
                "left = " +
                        ((Integer)armLeft.getTargetPosition()).toString() +
                        ", right = " +
                        ((Integer)armRight.getTargetPosition()).toString());
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
