package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.List;

public class RobotHardware {
    public LinearOpMode myOpMode;

    public RobotHardware(LinearOpMode opMode) {
        myOpMode = opMode;
    }
    //sensors



    // Motors
    public DcMotorEx fLeft;
    public DcMotorEx fRight;
    public DcMotorEx bLeft;
    public DcMotorEx bRight;
    public DcMotorEx extendo;
    public DcMotorEx hangMotor;
    public DcMotorEx lLift;
    public DcMotorEx rLift;

    //Sensors
    public TouchSensor touch1;
    public TouchSensor touch2;


    // Odometry
    public Encoder leftEncoder, rightEncoder, frontEncoder;


    // Servo for intake pitch control
    public Servo intakePitch;
//    public CRServoImpl roller;
    public Servo clawServo;
    public Servo wrist;
    public Servo armRight;
    public Servo armLeft;
    public Servo lock1;
    public Servo lock2;
    public Servo liftLock;
    public Servo sweeper;
    public Servo intakeWrist;
    public Servo clawIntake;
    public Servo fourBarPitch;

    //SubSystems Intake and others
    public Mecnum mecnum = new Mecnum();
    public Intake intake = new Intake();
    public Claw claw = new Claw();
    public Lifts lifts = new Lifts();
    public Hang hang = new Hang();
    Component[] components = {mecnum, intake, lifts, claw, hang};


    public void init() {
        // Initialize drive motors
        fLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "fLeft");
        fRight = myOpMode.hardwareMap.get(DcMotorEx.class, "fRight");
        bLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "bLeft");
        bRight = myOpMode.hardwareMap.get(DcMotorEx.class, "bRight");
        lLift = myOpMode.hardwareMap.get(DcMotorEx.class, "lLift");
        rLift = myOpMode.hardwareMap.get(DcMotorEx.class, "rLift");
        extendo = myOpMode.hardwareMap.get(DcMotorEx.class, "extendo");
        hangMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "hang");

        //Initialize servos
        //roller = myOpMode.hardwareMap.get(CRServoImpl.class, "roller");
        clawIntake = myOpMode.hardwareMap.get(Servo.class, "claw intake");
        intakePitch = myOpMode.hardwareMap.get(Servo.class, "intake pitch");
        clawServo = myOpMode.hardwareMap.get(Servo.class, "clawServo");
        wrist = myOpMode.hardwareMap.get(Servo.class, "wrist");
        armRight = myOpMode.hardwareMap.get(Servo.class, "arm right");
        armLeft = myOpMode.hardwareMap.get(Servo.class, "arm left");
        lock1 = myOpMode.hardwareMap.get(Servo.class, "lock1");
        lock2 = myOpMode.hardwareMap.get(Servo.class, "lock2");
        liftLock = myOpMode.hardwareMap.get(Servo.class, "liftLock");
        sweeper = myOpMode.hardwareMap.get(Servo.class, "sweeper");
        intakeWrist = myOpMode.hardwareMap.get(Servo.class, "intakeWrist");
        fourBarPitch = myOpMode.hardwareMap.get(Servo.class, "fourBar");

        //Initiallize sensors
        touch1 = myOpMode.hardwareMap.get(TouchSensor.class, "touch1");
        touch2 = myOpMode.hardwareMap.get(TouchSensor.class, "touch2");


        for (int i = 0; i < components.length; i++)
        {
            components[i].init(this);
        }

        myOpMode.telemetry.addData("status", "initialized");
        myOpMode.telemetry.update();
    }


}