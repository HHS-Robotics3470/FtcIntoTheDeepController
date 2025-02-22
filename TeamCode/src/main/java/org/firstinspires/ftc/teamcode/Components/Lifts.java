package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Lifts implements Component {
    private final int liftShift = HoldLastLift.getHeight();

    private final int LIFT_LOW = 0;
    private final int LIFT_HIGH = 4100;
    private final int LIFT_BACK = 0;
    private final int LIFT_FORWARD = 1810;
    private final int LIFT_SPECIMEN = 2700;
    private final int LIFT_TELEOP_SPECIMEN = 1600;
    private final int LIFT_BASKET = 3800;
    private final double LOCK_OPEN = 1;
    private final double LOCK_CLOSE = 0;
    public enum LIFT_STATE{
        INACTIVE,
        MOVING_HIGH,
        MOVING_LOW,
        MOVING_SPEC,
        MOVING_TELEOP,
    }
    private LIFT_STATE current_state = LIFT_STATE.INACTIVE;

    private LinearOpMode myOpMode = null;
    // Lift motors
    private DcMotorEx lLift;
    private DcMotorEx rLift;
    private Servo lock;


    public TouchSensor touch1;
    public TouchSensor touch2;

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
        touch1 = robotHardware.touch1;
        touch2 = robotHardware.touch2;

        lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
        lLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set motor direction for extendo (adjust as necessary)
        lLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rLift.setDirection(DcMotorSimple.Direction.FORWARD);
        extendo.setDirection(DcMotor.Direction.FORWARD);

//
        current_state = LIFT_STATE.INACTIVE;
        moveLiftsToZero();
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
        if (true) {
            rLift.setTargetPosition(LIFT_LOW);
            lLift.setTargetPosition(LIFT_LOW);
            rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rLift.setPower(1);
            lLift.setPower(1);
        }
    }

    // Stop Lift function
    public void stopLiftVertical() {
        rLift.setTargetPosition(rLift.getCurrentPosition());
        lLift.setTargetPosition(lLift.getCurrentPosition());
        lLift.setPower(0);
        rLift.setPower(0);
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

    public void GoToPositionVerticalSpecIntake()
    {
        int targetPosition = 0; // adjust for shtuff
        lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lLift.setTargetPosition(targetPosition);
        rLift.setTargetPosition(targetPosition);

        lLift.setPower(1.0);
        rLift.setPower(1.0);
    }
    public void GoToPositionVerticalSpecOuttake()
    {
        int targetPosition = 1600; // adjust for shtuff
        lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lLift.setTargetPosition(targetPosition);
        rLift.setTargetPosition(targetPosition);

        lLift.setPower(1.0);
        rLift.setPower(1.0);
    }

    public void setLiftPostion(int position) {
            lLift.setTargetPosition(position);
            rLift.setTargetPosition(position);

            lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            lLift.setPower(1.0);
            rLift.setPower(1.0);
    }

    public void GoToPositionHorizontal(int target){
        while (extendo.getCurrentPosition() != target)
        {
            extendo.setTargetPosition(target);


            extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            extendo.setPower(1);

        }
    }

    // New function to move lifts down to position 0 until a TouchSensor is pressed
    public void moveLiftsToZero() {
        lLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lLift.setPower(-1);
        rLift.setPower(-1);

        while (!touch1.isPressed() && !touch2.isPressed()) {
            // Wait until one of the touch sensors is pressed
        }
//
        lLift.setPower(0);
        rLift.setPower(0);

        lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public boolean isSortOfEqual(int lifts, int target, int error)
    {
        int difference = Math.abs(lifts - target);
        if (difference < error)
            return true;
        else
            return false;
    }
    public void ParallelMoveVertical(int target)
    {
        if (!isSortOfEqual(lLift.getCurrentPosition(), target, 10) && !isSortOfEqual(rLift.getCurrentPosition(), target, 10))
        {
            lLift.setTargetPosition(target);
            rLift.setTargetPosition(target);

//            lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            lLift.setPower(1);
            rLift.setPower(1);

            if (isSortOfEqual(lLift.getCurrentPosition(), target, 10) || isSortOfEqual(rLift.getCurrentPosition(), target, 10))
            {
                current_state = LIFT_STATE.INACTIVE;
            }
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
                lLift.setPower(0);
                rLift.setPower(0);
                break;
            case MOVING_LOW:
                ParallelMoveVertical(0);
                break;
            case MOVING_HIGH:
                ParallelMoveVertical(LIFT_BASKET);
                break;
            case MOVING_SPEC:
                ParallelMoveVertical(LIFT_SPECIMEN);
                break;
            case MOVING_TELEOP:
                ParallelMoveVertical(LIFT_TELEOP_SPECIMEN);


        }
    }

    public LIFT_STATE getCurrentState()
    {
        return current_state;
    }

    public void AutoHigh()
    {
        lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        current_state = LIFT_STATE.MOVING_HIGH;
    }

    public void AutoLow()
    {
        lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        current_state = LIFT_STATE.MOVING_LOW;
    }

    public void AutoSpec()
    {
        lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        current_state = LIFT_STATE.MOVING_SPEC;
    }

    public void TeleOpSpec() {
        lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        current_state = LIFT_STATE.MOVING_TELEOP;
    }

    public void AutoWait()
    {
        while (current_state != LIFT_STATE.INACTIVE)
        {
            stateUpdate();
        }
    }

    public boolean IsInactive()
    {
        return (current_state == LIFT_STATE.INACTIVE);
    }


}


