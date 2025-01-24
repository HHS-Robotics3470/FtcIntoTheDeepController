package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Hang implements Component{
    private DcMotorEx hangMotor;
    private Servo hangLock1;
    private Servo hangLock2;

    private final double LOCK_POS= 0.13;//0.07
    private final double UNLOCK_POS= 0.1;
    //Hang Position Tweaking
//
    @Override
    public void init(RobotHardware robotHardware) {
        hangMotor = robotHardware.hangMotor;
        hangLock1 = robotHardware.lock1;
        hangLock2 = robotHardware.lock2;

        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangLock2.setDirection(Servo.Direction.REVERSE);

        lock();
    }

    // Method to lift the robot
    public void lift() {
        hangMotor.setPower(1);
    }

    // Method to lower the robot
    public void lower() {
        hangMotor.setPower(-1);
    }

    // Method to stop the lift
    public void stop() {
        hangMotor.setPower(0);
    }

    // Method to lock the hang mechanism
    public void lock() {
        hangLock1.setPosition(LOCK_POS);
        hangLock2.setPosition(LOCK_POS);// Adjust based on servo mechanics
    }

    // Method to unlock the hang mechanism
    public void unlock() {
        hangLock1.setPosition(UNLOCK_POS);
        hangLock2.setPosition(UNLOCK_POS); // Adjust based on servo mechanics
    }

    public void toggle()
    {
        if (Math.abs(hangLock1.getPosition() - UNLOCK_POS) < 0.00001)
        {
            lock();
        }
        else
        {
            unlock();
        }
    }
}
