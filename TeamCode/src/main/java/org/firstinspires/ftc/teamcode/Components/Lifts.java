package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Lifts implements Component {
    private final int LIFT_LOW = 0;
    private final int LIFT_HIGH = 4100;
    private final int LIFT_BACK = 0;
    private final int LIFT_FORWARD = 1810;
    private final int LIFT_SPECIMEN = 1750;
    private final int LIFT_BASKET = 3800;
    private final double LOCK_OPEN = 1;
    private final double LOCK_CLOSE = 0;
    private enum LIFT_STATE{
        INACTIVE,
        MOVING_HIGH,
        MOVING_LOW,
        MOVING_SPEC,
    }
    private LIFT_STATE current_state = LIFT_STATE.INACTIVE;

    private LinearOpMode myOpMode = null;
    // Lift motors
    private DcMotorEx lLift;
    private DcMotorEx rLift;
    private Servo lock;

    // Horizontal extension motor
    public DcMotorEx extendo;

    // Init function
    public void init(RobotHardware robotHardware) {
        // Initialize lift motors from RobotHardware
        myOpMode = robotHardware.myOpMode;
        lLift = robotHardware.lLift;
        rLift = robotHardware.rLift;
        extendo = robotHardware.extendo;
        lock = robotHardware.liftLock;

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
        if (lLift.getCurrentPosition() < LIFT_HIGH && rLift.getCurrentPosition() < LIFT_HIGH) {
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
        if (rLift.getCurrentPosition() > LIFT_LOW && lLift.getCurrentPosition() > LIFT_LOW) {
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

    public void GoToPositionVertical(int target)
    {
        while (lLift.getCurrentPosition() != target && rLift.getCurrentPosition() != target)
        {
            lLift.setTargetPosition(target);
            rLift.setTargetPosition(target);

            lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            lLift.setPower(1);
            rLift.setPower(1);
        }
    }

    public void GoToPositionHorizontal(int target){
        while (extendo.getCurrentPosition() != target)
        {
            extendo.setTargetPosition(target);


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

    public void lockOpen()
    {
        lock.setPosition(LOCK_OPEN);
    }

    public void lockClose()
    {
        lock.setPosition(LOCK_CLOSE);
    }


//    // New function for horizontal extension
    public void extendHorizontally(RobotHardware robotHardware) {
        // Initialize extendo motor from RobotHardware
        extendo = robotHardware.extendo;

        // Reset encoder position for extendo
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set motor direction for extendo (adjust as necessary)
        extendo.setDirection(DcMotor.Direction.FORWARD);
    }

    // Function to set position for the extendo motor
    public void setExtendoPosition(int position, double power) {
        extendo.setTargetPosition(position);
        extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendo.setPower(power);
    }

    public void ParallelMoveVertical(int target)
    {
        if (lLift.getCurrentPosition() != target && rLift.getCurrentPosition() != target)
        {
            lLift.setTargetPosition(target);
            rLift.setTargetPosition(target);

            lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            lLift.setPower(1);
            rLift.setPower(1);
        }
        else
        {
            current_state = LIFT_STATE.INACTIVE;
        }
    }

    public void stateUpdate()
    {
        switch (current_state)
        {
            case INACTIVE:
                stopLiftVertical();
                break;
            case MOVING_LOW:
                ParallelMoveVertical(0);
                break;
            case MOVING_HIGH:
                ParallelMoveVertical(LIFT_BASKET);
            case MOVING_SPEC:
                ParallelMoveVertical(LIFT_SPECIMEN);

        }
    }

    public LIFT_STATE getCurrentState()
    {
        return current_state;
    }

    public void AutoHigh()
    {
        current_state = LIFT_STATE.MOVING_HIGH;
    }

    public void AutoLow()
    {
        current_state = LIFT_STATE.MOVING_LOW;
    }

    public void AutoSpec()
    {
        current_state = LIFT_STATE.MOVING_SPEC;
    }




}
