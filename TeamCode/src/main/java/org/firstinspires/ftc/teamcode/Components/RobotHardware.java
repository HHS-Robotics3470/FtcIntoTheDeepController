package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.List;

public class RobotHardware {
    private LinearOpMode myOpMode = null;

    public RobotHardware(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    public double driveSpeedControl = 0.8;

    // Motors
    public DcMotorEx fLeft;
    public DcMotorEx fRight;
    public DcMotorEx bLeft;
    public DcMotorEx bRight;

    // Lift Motors
    public DcMotor lLift;
    public DcMotor rLift;
    public DcMotor intake;


    // Odometry
    public Encoder leftEncoder, rightEncoder, frontEncoder;
    public StandardTrackingWheelLocalizer localizer;

    public void init() {
        // Initialize drive motors
        fLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "fLeft");
        fRight = myOpMode.hardwareMap.get(DcMotorEx.class, "fRight");
        bLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "bLeft");
        bRight = myOpMode.hardwareMap.get(DcMotorEx.class, "bRight");

        // Initialize lift motors
        lLift = myOpMode.hardwareMap.get(DcMotor.class, "lLift");
        rLift = myOpMode.hardwareMap.get(DcMotor.class, "rLift");

        intake = myOpMode.hardwareMap.get(DcMotor.class, "intakey");

        // Initialize encoders
        leftEncoder = new Encoder(myOpMode.hardwareMap.get(DcMotorEx.class, "fLeft"));
        rightEncoder = new Encoder(myOpMode.hardwareMap.get(DcMotorEx.class, "fRight"));
        frontEncoder = new Encoder(myOpMode.hardwareMap.get(DcMotorEx.class, "bLeft"));

        // Initialize localizer
        List<Integer> initalpos = new ArrayList<>();
        List<Integer> initalvel = new ArrayList<>();
        for (int i = 0; i < 2; i++) {
            initalpos.add(0);
            initalvel.add(0);
        }
        localizer = new StandardTrackingWheelLocalizer(myOpMode.hardwareMap, initalpos, initalvel);

        myOpMode.telemetry.addData("status", "initialized");
        myOpMode.telemetry.update();
    }
}
