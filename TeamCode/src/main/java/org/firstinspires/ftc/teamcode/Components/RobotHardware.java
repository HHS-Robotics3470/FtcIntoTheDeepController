package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServoImpl;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.List;

public class RobotHardware {
    public LinearOpMode myOpMode = null;

    public RobotHardware(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    // Motors
    public DcMotorEx fLeft;
    public DcMotorEx fRight;
    public DcMotorEx bLeft;
    public DcMotorEx bRight;
    public DcMotorEx extendo;
    public DcMotorEx hangMotor;


    // Lift Motors
    public DcMotor lLift;
    public DcMotor rLift;


    // Odometry
    public Encoder leftEncoder, rightEncoder, frontEncoder;
    public StandardTrackingWheelLocalizer localizer;


    // Servo for intake pitch control
    public Servo intakePitch;
    public CRServoImpl roller;
    public Servo clawServo;
    public Servo wrist;
    public Servo armRight;
    public Servo armLeft;
    public Servo lock1;
    public Servo lock2;

    //SubSystems
    public Mecnum mecnum = new Mecnum();
    public Intake intake = new Intake();
    public Claw claw = new Claw();
    public Lifts lifts = new Lifts();
    public Hang hang = new Hang();
    Component[] components = {mecnum, intake, claw, lifts, hang};


    public void init() {
        // Initialize drive motors
        fLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "fLeft");
        fRight = myOpMode.hardwareMap.get(DcMotorEx.class, "fRight");
        bLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "bLeft");
        bRight = myOpMode.hardwareMap.get(DcMotorEx.class, "bRight");

        // Initialize lift motors
        lLift = myOpMode.hardwareMap.get(DcMotor.class, "lLift");
        rLift = myOpMode.hardwareMap.get(DcMotor.class, "rLift");

        roller = myOpMode.hardwareMap.get(CRServoImpl.class, "intakey");

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

        for (int i = 0; i < components.length; i++)
        {
            components[i].init(this);
        }

        myOpMode.telemetry.addData("status", "initialized");
        myOpMode.telemetry.update();
    }
}
