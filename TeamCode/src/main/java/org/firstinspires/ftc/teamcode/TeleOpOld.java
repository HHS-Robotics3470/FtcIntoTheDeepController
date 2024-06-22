package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="TestOpMode", group="blah")
public class TeleOpOld extends LinearOpMode {

    robotHardwareOld robot = new robotHardwareOld(this);
    public boolean ifLaunched = false;
    public boolean ifLifted = false;
    public boolean xState = false;
    public boolean y2State = false;
    public boolean b2State = false;
    public boolean bState = false;
    public boolean aState = false;
    public boolean a2State = false;
    public boolean padLeft = false;
    public boolean ifOpen = false;
    public boolean ifClawOpened = false;
    public boolean ifRolled = false;
    public boolean ifHooked = false;
    public boolean rstate = false;
    public boolean ifDrop = false;
    public int heightState = 0;
    public boolean dpadState = false;
    public boolean dpadState2b = false;
    public int markerNum = 0;
    public boolean x1State = false;
    public boolean clawDrop = false;







    public String currentPos(Gamepad gamepad1)
    {
        return "robot.setMovementPosition("+gamepad1.left_stick_y+","+gamepad1.left_stick_x+","+gamepad1.right_stick_x+","+robot.fLeft.getCurrentPosition()+","+robot.fRight.getCurrentPosition()+
                ","+robot.bLeft.getCurrentPosition()+","+robot.bRight.getCurrentPosition()+","+robot.rLift.getCurrentPosition()+","+robot.lLift.getCurrentPosition()+","+
                ifDrop +","+robot.flipper.getPosition()+","+robot.gears.getPosition()+","+robot.claw.getPosition()+","+robot.arm.getPosition()+","+robot.wrist.getPosition()+","+clawDrop+",ifMirror);";
    }
    @Override
    public void runOpMode() {

        robot.init();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("status", "started");
//            telemetry.addData("Left Distance Sensor", robot.leftSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right Distance Sensor", robot.rightSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Gears Servo Position", robot.gears.getPosition());
            telemetry.addData("Dropper Servo Position", robot.dropper.getPosition());
            telemetry.addData("Bomber Servo Position", robot.bomber.getPosition());
            telemetry.addData("Mover Servo Position", robot.flipper.getPosition());
            telemetry.addData("Hook Servo Position", robot.hook.getPosition());
            telemetry.addData("Claw Servo Position", robot.claw.getPosition());
            telemetry.addData("Roller Servo Position", robot.arm.getPosition());
            telemetry.addData("Wrist Servo Position", robot.wrist.getPosition());
            telemetry.addData("llift", robot.lLift.getCurrentPosition());
            telemetry.addData("llift", robot.lLift.getCurrentPosition());
            telemetry.addData("height", heightState);
            telemetry.update();


/*
            if (robot.touchSensor.isPressed()) {
                telemetry.addData("Touch Sensor", "Is Pressed");
            } else {
                telemetry.addData("Touch Sensor", "Is Not Pressed");
            }
            telemetry.update();
            if
*/
            //instanciation and then passing the control through
            robot.driveRobot(gamepad1);


            //lift code
            if (gamepad2.right_bumper) {
                robot.lowerLift();
            } else if (gamepad2.left_bumper) {
                robot.raiseLift();
            } else {
                robot.stopLift();
            }

            //call to roller function if button pressed
            if (gamepad2.dpad_up){
                robot.winchUp(gamepad2.dpad_up);
            }
            else if(gamepad2.dpad_down){
                robot.winchDown(gamepad2.dpad_down);
            }
            else if(!gamepad2.dpad_up) {
                robot.winchUp(gamepad2.dpad_up);
            }

            if (gamepad1.left_bumper){
                robot.intakeRolling(gamepad1.left_bumper);
            }
            else   {
                robot.intakeRolling(false);
            }

            //have to add reverse roller function
            if (gamepad1.right_bumper){
                robot.firstDrop(gamepad1.right_bumper);
            }

            //sets adjusting positions
            if (gamepad1.y){
                robot.resetClaw(gamepad1.y);
                robot.adjusting((gamepad1.y));
            }

            if (gamepad1.a && !aState) {


                ifClawOpened = robot.useClaw(ifClawOpened);
                aState = true;
            } else if (!gamepad1.a && aState) {
                aState = false;
            }



            if (gamepad2.a && !a2State) {

                robot.useArm(ifRolled);
                ifRolled = !ifRolled;
                heightState = 0;
                a2State = true;
            } else if (!gamepad2.a && a2State) {
                a2State = false;
            }

            if (gamepad1.dpad_up && !dpadState) {


                heightState = robot.UpHeights(heightState);
                dpadState = true;
            } else if (!gamepad1.dpad_up && dpadState) {
                dpadState = false;
            }

            if (gamepad1.dpad_down && !dpadState2b) {


                heightState = robot.DownHeights(heightState);
                dpadState2b = true;
            } else if (!gamepad1.dpad_down && dpadState2b) {
                dpadState2b = false;
            }

            if (gamepad2.dpad_left && !padLeft) {
                robot.arm.setPosition(robot.rollDriving);
                padLeft = true;
            } else if (!gamepad2.dpad_left && padLeft) {
                padLeft = false;
            }



            if (gamepad2.y && !y2State) {

                robot.nine_eleven(ifLaunched);
                ifLaunched = !ifLaunched;
                y2State = true;
            } else if (!gamepad2.y && y2State) {
                y2State = false;
            }

            if (gamepad2.b && !b2State) {

                robot.hookRobot(ifHooked);
                ifHooked = !ifHooked;
                b2State = true;
            } else if (!gamepad2.b && b2State) {
                b2State = false;
            }


            if (gamepad2.x && !xState) {
                robot.resetClaw(ifLifted);
                robot.lift_pixel(ifLifted);
                ifLifted = !ifLifted;
                xState = true;
            } else if (!gamepad2.x && xState) {
                xState = false;
            }

            clawDrop = false;
            if (gamepad1.x && !x1State) {
                robot.groundDrop();
                clawDrop = true;
                x1State = true;
            } else if (!gamepad1.x && x1State) {
                x1State = false;
            }



        }


    }
}