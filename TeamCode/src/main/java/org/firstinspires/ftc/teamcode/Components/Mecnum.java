package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Mecnum implements Component {
    private RobotHardware robotHardware;

    public final double DRIVE_SPEED_MAX = 0.5;
    public final double DRIVE_SPEED_SLOW = 0.05;
    public double driveSpeedControl = DRIVE_SPEED_MAX;

    // Custom speeds for individual motors
    public double speedFLeft = 1.0;
    public double speedFRight = 1.0;
    public double speedBLeft = 1.0; //Back should have more power
    public double speedBRight = 1.0; //Back should have more power

    // Motors
    public DcMotorEx fLeft;
    public DcMotorEx fRight;
    public DcMotorEx bLeft;
    public DcMotorEx bRight;

    @Override
    public void init(RobotHardware robotHardware) {
        fLeft = robotHardware.fLeft;
        fRight = robotHardware.fRight;
        bLeft = robotHardware.bLeft;
        bRight = robotHardware.bRight;

        fLeft.setDirection(DcMotorEx.Direction.REVERSE);
        fRight.setDirection(DcMotorEx.Direction.FORWARD);
        bLeft.setDirection(DcMotorEx.Direction.REVERSE);
        bRight.setDirection(DcMotorEx.Direction.FORWARD);

        fLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Separate drive controls for each motor with individual speeds
    public void setDrivePowerFLeft(double power) {
        fLeft.setPower(power * driveSpeedControl * speedFLeft);
    }

    public void setDrivePowerFRight(double power) {
        fRight.setPower(power * driveSpeedControl * speedFRight);
    }

    public void setDrivePowerBLeft(double power) {
        bLeft.setPower(power * driveSpeedControl * speedBLeft);
    }

    public void setDrivePowerBRight(double power) {
        bRight.setPower(power * driveSpeedControl * speedBRight);
    }

    // Unified method for all motors
    public void setDrivePowerAuto(double v1, double v2, double v3, double v4) {
        setDrivePowerFLeft(v1);
        setDrivePowerFRight(v2);
        setDrivePowerBLeft(v3);
        setDrivePowerBRight(v4);
    }

    public void setDrivePower(double v1, double v2, double v3, double v4) {

        fLeft.setPower(v1 * driveSpeedControl);
        fRight.setPower(v2 * driveSpeedControl);
        bLeft.setPower(v3 * driveSpeedControl);
        bRight.setPower(v4 * driveSpeedControl);
    }

    // Power function for Auto
    public void powerFunction(double speedF, double speedR, double speedB, double speedFL) {
        this.speedFLeft = speedFL;
        this.speedFRight = speedF;
        this.speedBLeft = speedB;
        this.speedBRight = speedR;
    }

    public void driveRobot(Gamepad gamepad1) {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double frLeft = y + x + rx;
        double frRight = y - x - rx;
        double baLeft = y - x + rx;
        double baRight = y + x - rx;

        setDrivePower(frLeft, frRight, baLeft, baRight);
    }

    public void brake(double button) {
//        if (button)
//        {
//            driveSpeedControl = DRIVE_SPEED_SLOW;
//        }
//        else
//        {
//            driveSpeedControl = DRIVE_SPEED_MAX;
//        };
//        driveSpeedControl = (DRIVE_SPEED_MAX * button) + DRIVE_SPEED_SLOW;
    }

    /**
     * Move the robot forward for a specific duration.
     *
     * @param power the power level to set for the motors
     * @param time      the duration to move in milliseconds
     */
    private void moveForward(double power, int time) {
        setDrivePower(power, power, power, power);
        stopMoving();
    }

    /**
     * Stop all drive motors.
     */
    private void stopMoving() {
        robotHardware.fLeft.setPower(0);
        robotHardware.fRight.setPower(0);
        robotHardware.bLeft.setPower(0);
        robotHardware.bRight.setPower(0);
    }

    /**
     * Strafe the robot either left or right.
     *
     * @param power the power level to set for the motors
     * @param time      the duration to move in milliseconds
     */
    public void strafe(double power, int time) {
        robotHardware.fLeft.setPower(power * driveSpeedControl * speedFLeft);
        robotHardware.fRight.setPower(-power * driveSpeedControl * speedFRight);
        robotHardware.bLeft.setPower(-power * driveSpeedControl * speedBLeft);
        robotHardware.bRight.setPower(power * driveSpeedControl * speedBRight);
        stopMoving();
    }
//
    /**
     * Rotate the robot either left or right.
     *
     * @param power the power level to set for the motors
     * @param time      the duration to rotate in milliseconds
     */
    public void rotate(double power, int time) {
        robotHardware.fLeft.setPower(power * driveSpeedControl * speedFLeft);
        robotHardware.fRight.setPower(-power * driveSpeedControl * speedFRight);
        robotHardware.bLeft.setPower(power * driveSpeedControl * speedBLeft);
        robotHardware.bRight.setPower(-power * driveSpeedControl * speedBRight);
        stopMoving();
    }
}



