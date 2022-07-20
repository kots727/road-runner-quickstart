package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.libs.*;

public class ButterDriveWPI {
    double width = .12344*2;//in meters
    double length = .1862*2;//in meters
    public static final double WHEEL_CIRCUMFERENCE = (4*0.0254) * Math.PI;
    public static final double TICKS_PER_ROTATION = 1120;
    public static final double MAX_TICKS_PER_SECOND = 2500;
public static double backmult = 312/435;
    public static final double CHASSIS_RAD_SQUARED = (7*.0254)*(7*.0254) + (8*.0254)*(8*.0254);
    public static final double MAX_DRIVE_SPEED = MAX_TICKS_PER_SECOND * WHEEL_CIRCUMFERENCE / TICKS_PER_ROTATION;
    public static final double MAX_ANGULAR_SPEED = MAX_DRIVE_SPEED / Math.sqrt(CHASSIS_RAD_SQUARED);

    DcMotor[] motors = new DcMotor[4];
    org.firstinspires.ftc.teamcode.libs.Translation2d m_frontLeftLocation =new org.firstinspires.ftc.teamcode.libs.Translation2d(width/2, length/2);
    org.firstinspires.ftc.teamcode.libs.Translation2d m_frontRightLocation =new org.firstinspires.ftc.teamcode.libs.Translation2d(width/2, -length/2);
    org.firstinspires.ftc.teamcode.libs.Translation2d m_backLeftLocation =new org.firstinspires.ftc.teamcode.libs.Translation2d(width/2, -length/2);
    org.firstinspires.ftc.teamcode.libs.Translation2d m_backRightLocation =new org.firstinspires.ftc.teamcode.libs.Translation2d(width/2, length/2);
    org.firstinspires.ftc.teamcode.libs.MecanumDriveKinematics m_kinematics = new org.firstinspires.ftc.teamcode.libs.MecanumDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

public void drive(double vx, double vy, double va, double imu,boolean ftp){
    org.firstinspires.ftc.teamcode.libs.ChassisSpeeds speeds = org.firstinspires.ftc.teamcode.libs.ChassisSpeeds.fromFieldRelativeSpeeds(
            vx, vy, va, org.firstinspires.ftc.teamcode.libs.Rotation2d.fromRadians(imu)
    );
   /* if(ftp){
        speeds.vyzero();
    }
*/
    MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
    wheelSpeeds.desaturate(MAX_DRIVE_SPEED);
    // Convert to module states
    double frontLeft = wheelSpeeds.frontLeftMetersPerSecond;
    double frontRight = wheelSpeeds.frontRightMetersPerSecond;
    double backLeft = wheelSpeeds.rearLeftMetersPerSecond;
    double backRight = wheelSpeeds.rearRightMetersPerSecond;
    motors[0].setPower((backLeft/MAX_DRIVE_SPEED));
    motors[1].setPower(frontLeft/MAX_DRIVE_SPEED);
    motors[2].setPower(frontRight/MAX_DRIVE_SPEED);
    motors[3].setPower((backRight/MAX_DRIVE_SPEED));
}

    public void init(HardwareMap hardwareMap) {
        String[] motorNames = new String[]{"back_left_motor", "front_left_motor", "front_right_motor", "back_right_motor"};
        for (int i = 0; i < 4; i++)
            motors[i] = (DcMotor) hardwareMap.get(DcMotor.class, motorNames[i]);
        //motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        //motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        //motors[3].setDirection(DcMotorSimple.Direction.REVERSE);
    }



}
