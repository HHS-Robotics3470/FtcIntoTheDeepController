package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;


public class ExampleRobotHardware {
    private LinearOpMode myOpMode = null;
    public ExampleRobotHardware(LinearOpMode opMode){
        myOpMode = opMode;
    }

    public double driveSpeedControl = 0.8;

    //Moters
    public DcMotor fLeft;
    public DcMotor fRight;
    public DcMotor bLeft;
    public DcMotor bRight;
    public DcMotor lLift;
    public DcMotor rLift;
    public DcMotor intake;

    //Servos
    public Servo servo;

    //Sensors
    public DistanceSensor rightSensor;

    //Servo positions
    public final double servoActive = 0.0398;
    public final double servoInactive = 0.049;

    //Moter positions
    public final double liftAbove = -1390;

    //Servo States
    public boolean servoState = false;
    public void init(){

//        MOTORS
        fLeft = myOpMode.hardwareMap.get(DcMotor.class, "fLeft");
        fRight = myOpMode.hardwareMap.get(DcMotor.class, "fRight");
        bLeft = myOpMode.hardwareMap.get(DcMotor.class, "bLeft");
        bRight = myOpMode.hardwareMap.get(DcMotor.class, "bRight");
        lLift = myOpMode.hardwareMap.get(DcMotor.class, "lLift");
        rLift = myOpMode.hardwareMap.get(DcMotor.class, "rLift");

        lLift = myOpMode.hardwareMap.get(DcMotor.class, "lLift");
        rLift = myOpMode.hardwareMap.get(DcMotor.class, "rLift");

        intake = myOpMode.hardwareMap.get(DcMotor.class, "intakey");

        //Servos
        servo = myOpMode.hardwareMap.get(Servo.class, "claw");

        //Distance sensors
//        leftSensor = myOpMode.hardwareMap.get(DistanceSensor.class, "distance2");
        rightSensor = myOpMode.hardwareMap.get(DistanceSensor.class, "distance1");


        //Direction and encoders
        fLeft.setDirection(DcMotor.Direction.FORWARD);
        fRight.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.REVERSE);

        lLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rLift.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);


        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        myOpMode.telemetry.addData( "status", "initialized");
        myOpMode.telemetry.update();
        myOpMode.waitForStart();

        //Servo initial positions
        servo.setPosition(servoInactive);


    }

    //Mechnam code for robot
    public void driveRobot(Gamepad gamepad1){
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
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


    public void raiseLift() {
        if (lLift.getCurrentPosition() < 0) {
            rLift.setTargetPosition(0);
            lLift.setTargetPosition(0);
            rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rLift.setPower(1);
            lLift.setPower(1);
        }
    }

    public void lowerLift() {
        if (rLift.getCurrentPosition() > -4250) { //if not touching button go back until it does, then reset your encoders
            rLift.setTargetPosition(-4275);
            lLift.setTargetPosition(-4275);
            rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rLift.setPower(-1);
            lLift.setPower(-1);
        }
    }

    public void stopLift() {
        rLift.setTargetPosition(rLift.getCurrentPosition());
        lLift.setTargetPosition(lLift.getCurrentPosition());
    }

    public void ServoFunction1()
    {
        servo.setPosition(servoActive);
    }

    public void ServoFunction2()
    {
        servoState = SwitchServo(servo, servoActive, servoInactive, servoState);
    }

    public void ServoFunction3(boolean open)
    {
        DelayServo(servo, servoActive, servoInactive, 200);
    }

    //swiches a servo between two positions
    public boolean SwitchServo(Servo s, double active, double inactive, boolean isActive)
    {
        if (!isActive)
            s.setPosition(active);
        else
            s.setPosition(inactive);
        return !isActive;
    }

    //moves servo to a certain position, then after a delay, moves it back to the previous position position
    public void DelayServo(Servo s, double active, double inactive, int milliseconds)
    {
        s.setPosition(active);
        myOpMode.sleep(milliseconds);
        s.setPosition(inactive);
    }





}