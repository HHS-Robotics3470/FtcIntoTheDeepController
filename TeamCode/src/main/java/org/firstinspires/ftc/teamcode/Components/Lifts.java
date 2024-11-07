package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Lifts implements Component {
    private final int LIFT_LOW = 0;
    private final int LIFT_HIGH = 4275;
    private final int LIFT_BACK = 0;
    private final int LIFT_FORWARD = 4275;

    private LinearOpMode myOpMode = null;
    // Lift motors
    private DcMotor lLift;
    private DcMotor rLift;

    // Horizontal extension motor
    public DcMotorEx extendo;

    // Init function
    public void init(RobotHardware robotHardware) {
        // Initialize lift motors from RobotHardware
        myOpMode = robotHardware.myOpMode;
        lLift = robotHardware.lLift;
        rLift = robotHardware.rLift;
        extendo = robotHardware.extendo;

        lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set motor direction for extendo (adjust as necessary)
        lLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rLift.setDirection(DcMotorSimple.Direction.FORWARD);
        extendo.setDirection(DcMotor.Direction.FORWARD);
    }

    // Raise Lift function
    public void raiseLift() {
        if (lLift.getCurrentPosition() < LIFT_HIGH) {
            rLift.setTargetPosition(LIFT_HIGH);
            lLift.setTargetPosition(LIFT_HIGH);
            rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rLift.setPower(1);
            lLift.setPower(1);
        }
    }

    // Lower Lift function
    public void lowerLift() {
        if (rLift.getCurrentPosition() > LIFT_LOW) {
            rLift.setTargetPosition(LIFT_LOW);
            lLift.setTargetPosition(LIFT_LOW);
            rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rLift.setPower(-1);
            lLift.setPower(-1);
        }
    }

    // Stop Lift function
    public void stopLiftVertical() {
        rLift.setTargetPosition(rLift.getCurrentPosition());
        lLift.setTargetPosition(lLift.getCurrentPosition());
    }

    public void forwardLift() {
        if (extendo.getCurrentPosition() < LIFT_FORWARD) {
            extendo.setTargetPosition(LIFT_FORWARD);
            extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extendo.setPower(1);
        }
    }

    // Lower Lift function
    public void backLift() {
        if (extendo.getCurrentPosition() > LIFT_BACK) {
            extendo.setTargetPosition(LIFT_BACK);
            extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extendo.setPower(-1);
        }
    }

    // Stop Lift function
    public void stopLiftHorizontal() {
        extendo.setTargetPosition(extendo.getCurrentPosition());
    }


//    // New function for horizontal extension
//    public void extendHorizontally(RobotHardware robotHardware) {
//        // Initialize extendo motor from RobotHardware
//        extendo = robotHardware.extendo;
//
//        // Reset encoder position for extendo
//        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // Set motor direction for extendo (adjust as necessary)
//        extendo.setDirection(DcMotor.Direction.FORWARD);
//    }
//
//    // Function to set position for the extendo motor
//    public void setExtendoPosition(int position, double power) {
//        extendo.setTargetPosition(position);
//        extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        extendo.setPower(power);
//    }
}
