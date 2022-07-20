package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
@TeleOp(name = "Butterfly Complete", group = "Test")
public class ButteryTeleOp extends LinearOpMode {
    ButterDriveWPI bot = new ButterDriveWPI();
    public static double back_left_servo_inactive = 0;
    public static double back_left_servo_active = .5;
    public static double back_right_servo_inactive = .95;
    public static double back_right_servo_active = .38;
    public static double front_left_servo_inactive = 0;
    public static double front_left_servo_active = 0;
    public static double front_right_servo_inactive = .2;
    public static double front_right_servo_active =.7;
            BNO055IMU imu;
    boolean butteryMode = false;
    boolean butteryModeButtonChanged = false;
    public void runOpMode(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        bot.init(hardwareMap);
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";
        String[] servoNames = new String[]{"back_left_servo", "front_left_servo", "front_right_servo", "back_right_servo"};
        Servo[] servos = new Servo[4];
        for (int i = 0; i < 4; i++)
            servos[i] = (Servo) hardwareMap.get(Servo.class, servoNames[i]);
        this.imu.initialize(parameters);
        waitForStart();
        while(opModeIsActive()) {
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
            double vx = -gamepad1.left_stick_x * ButterDriveWPI.MAX_DRIVE_SPEED;
            double vy = -gamepad1.left_stick_y * ButterDriveWPI.MAX_DRIVE_SPEED;
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
            double va = gamepad1.right_stick_x * ButterDriveWPI.MAX_ANGULAR_SPEED;
            //float va = -gamepad1.right_stick_x;
            Orientation orientation = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            telemetry.addData("Heading", Math.toDegrees(orientation.firstAngle));
            telemetry.addData("vx,vy,va",vx+" , "+vy + " , "+va);
            telemetry.update();
            bot.drive(vy, vx, va,orientation.firstAngle,false);
        }
    }

}
