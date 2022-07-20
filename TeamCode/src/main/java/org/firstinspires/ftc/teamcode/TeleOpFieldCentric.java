package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@Config
@TeleOp(group = "advanced")
public class TeleOpFieldCentric extends LinearOpMode {
    public static double back_left_servo_inactive = 0;
    public static double back_left_servo_active = .25;
    public static double back_right_servo_inactive = .95;
    public static double back_right_servo_active = .65;
    public static double front_left_servo_inactive = 0;
    public static double front_left_servo_active = 0.3;
    public static double front_right_servo_inactive = .15;
    public static double front_right_servo_active =.4;
    @Override

    public void runOpMode() throws InterruptedException {

        String[] servoNames = new String[]{"back_left_servo", "front_left_servo", "front_right_servo", "back_right_servo"};
        Servo[] servos = new Servo[4];
        for (int i = 0; i < 4; i++)
            servos[i] = (Servo) hardwareMap.get(Servo.class, servoNames[i]);
        boolean butteryMode = false;
        boolean butteryModeButtonChanged = false;
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            if(gamepad1.a){
                if(butteryModeButtonChanged){}
                else {
                    butteryMode = !butteryMode;
                    butteryModeButtonChanged = true;
                }
            }
            else{
                butteryModeButtonChanged = false;
            }

            if(butteryMode) {
                servos[0].setPosition(back_left_servo_active);
                servos[1].setPosition(front_left_servo_active);
                servos[2].setPosition(front_right_servo_active);
                servos[3].setPosition(back_right_servo_active);
            }
            else{
                servos[0].setPosition(back_left_servo_inactive);
                servos[1].setPosition(front_left_servo_inactive);
                servos[2].setPosition(front_right_servo_inactive);
                servos[3].setPosition(back_right_servo_inactive);
            }
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -Math.pow(gamepad1.left_stick_y,3),
                    -Math.pow(gamepad1.left_stick_x,3)
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            if(butteryMode==true){
                drive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                               0,
                                -Math.pow(gamepad1.right_stick_x,3)
                        )
                );
            }
            else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -Math.pow(gamepad1.right_stick_x,3)
                        )
                );
            }
            // Update everything. Odometry. Etc.
            drive.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("eijaoi",gamepad1.right_stick_x +"  "+gamepad1.right_stick_y);
            telemetry.addData("fs",butteryMode);
            telemetry.update();
        }
    }
}