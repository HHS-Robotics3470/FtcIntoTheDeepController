package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.List;

public class Mecnum implements Component{


    public final double DRIVE_SPEED_MAX = 0.5;
    public final double DRIVE_SPEED_SLOW = 0.05;
    public double driveSpeedControl = DRIVE_SPEED_MAX;

    //Moters
    public DcMotorEx fLeft;
    public DcMotorEx fRight;
    public DcMotorEx bLeft;
    public DcMotorEx bRight;

    //Localizer
    public  StandardTrackingWheelLocalizer localizer;

    @Override
    public void init(RobotHardware robotHardware) {
        fLeft = robotHardware.fLeft;
        fRight = robotHardware.fRight;
        bLeft = robotHardware.bLeft;
        bRight = robotHardware.bRight;
        localizer = robotHardware.localizer;

        fLeft.setDirection(DcMotor.Direction.REVERSE);
        fRight.setDirection(DcMotor.Direction.FORWARD);
        bLeft.setDirection(DcMotor.Direction.REVERSE);
        bRight.setDirection(DcMotor.Direction.FORWARD);

        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Mechnam code for robot
    public void driveRobot(Gamepad gamepad1){
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double frLeft = y + x + rx;
        double frRight = y - x - rx;
        double baLeft = y - x + rx;
        double baRight = y + x - rx;

        setDrivePower(frLeft, frRight, baLeft, baRight);

    }

    public void fieldCentricDrve(Gamepad gamepad1){
        Pose2d poseEstimate = localizer.getPoseEstimate();

        Vector2d vector = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y).rotated(-poseEstimate.getHeading());

        double y = vector.getY();
        double x = vector.getX();
        double rx = -gamepad1.right_stick_x;

        double frLeft = y + x + rx;
        double frRight = y - x - rx;
        double baLeft = y - x + rx;
        double baRight = y + x - rx;

        setDrivePower(frLeft, frRight, baLeft, baRight);

    }

    //Sets driving speed
    public void setDrivePower(double v1, double v2, double v3, double v4) {

        fLeft.setPower(v1 * driveSpeedControl);
        fRight.setPower(v2 * driveSpeedControl);
        bLeft.setPower(v3 * driveSpeedControl);
        bRight.setPower(v4 * driveSpeedControl);
    }

    public void brake(double button)
    {
//        if (button)
//        {
//            driveSpeedControl = DRIVE_SPEED_SLOW;
//        }
//        else
//        {
//            driveSpeedControl = DRIVE_SPEED_MAX;
//        };
        driveSpeedControl = (DRIVE_SPEED_MAX * button) + DRIVE_SPEED_SLOW;
    }

}
