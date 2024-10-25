package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Components.Intake;

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
    public DcMotor lLift;
    public DcMotor rLift;
    public DcMotor roller;

    // Odometry
    public Encoder leftEncoder, rightEncoder, frontEncoder;
    public StandardTrackingWheelLocalizer localizer;

    //Sensors


    //Components
    public Intake intake = new Intake();
    public Mecnum mecnum = new Mecnum();
    public Component[] components = {intake, mecnum};

    public void init() {
        // Map moters
        fLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "fLeft");
        fRight = myOpMode.hardwareMap.get(DcMotorEx.class, "fRight");
        bLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "bLeft");
        bRight = myOpMode.hardwareMap.get(DcMotorEx.class, "bRight");
        lLift = myOpMode.hardwareMap.get(DcMotor.class, "lLift");
        rLift = myOpMode.hardwareMap.get(DcMotor.class, "rLift");
        roller = myOpMode.hardwareMap.get(DcMotor.class, "intakey");

        // Map encoders
        leftEncoder = new Encoder(myOpMode.hardwareMap.get(DcMotorEx.class, "fLeft"));
        rightEncoder = new Encoder(myOpMode.hardwareMap.get(DcMotorEx.class, "fRight"));
        frontEncoder = new Encoder(myOpMode.hardwareMap.get(DcMotorEx.class, "bLeft"));

        // Map sensors


        // Initialize localizer
        List<Integer> initalpos = new ArrayList<>();
        List<Integer> initalvel = new ArrayList<>();
        for (int i = 0; i < 2; i++) {
            initalpos.add(0);
            initalvel.add(0);
        }
        localizer = new StandardTrackingWheelLocalizer(myOpMode.hardwareMap, initalpos, initalvel);

        for (int i = 0; i < components.length; i++)
        {
            components[i].init(this);
        }


        myOpMode.telemetry.addData("status", "initialized");
        myOpMode.telemetry.update();
    }
}
