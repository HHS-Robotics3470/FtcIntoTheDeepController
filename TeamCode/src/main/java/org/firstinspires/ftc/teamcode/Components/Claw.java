package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Claw implements Component {
    private LinearOpMode myOpMode;
    private Servo clawServo;
    private Servo armRight;
    private Servo armLeft;
    private Servo wrist;
    private Servo hang;

    private boolean ifSwinged = false;
    private boolean ifSpecimen = false;

    private final ElapsedTime timer = new ElapsedTime();

    // Constants for servo positions
    private final double CLAW_OPEN_POSITION = 0.056;
    private final double CLAW_CLOSE_POSITION = 0;
    private final double ARM_UP_POSITION = 0.325;
    private final double ARM_DOWN_POSITION = 0.245;
    private final double ARM_REST_POSITION = 0.26;
    private final double WRIST_UP_POSITION = 0.35;
    private final double WRIST_DOWN_POSITION = 0.18;

    private Thread grabThread;
    private volatile boolean grabRunning = false;

    @Override
    public void init(RobotHardware robotHardware) {
        myOpMode = robotHardware.myOpMode;
        wrist = robotHardware.wrist;
        clawServo = robotHardware.clawServo;
        armRight = robotHardware.armRight;
        armLeft = robotHardware.armLeft;
        hang = robotHardware.lock1;

        hang.setDirection(Servo.Direction.REVERSE);
        armRight.setDirection(Servo.Direction.REVERSE);
        armLeft.setDirection(Servo.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.REVERSE);

        clawServo.setPosition(CLAW_CLOSE_POSITION);
        armRight.setPosition(ARM_REST_POSITION);
        armLeft.setPosition(ARM_REST_POSITION);
        wrist.setPosition(WRIST_DOWN_POSITION);

        ifSwinged = false;
        ifSpecimen = false;
    }

    // Claw methods
    public void clawOpen() {
        clawServo.setPosition(CLAW_OPEN_POSITION);
    }

    public void clawClose() {
        clawServo.setPosition(CLAW_CLOSE_POSITION);
    }

    // Arm methods
    public void armUp() {
        armRight.setPosition(ARM_UP_POSITION);
        armLeft.setPosition(ARM_UP_POSITION);
    }

    public void armDown() {
        armRight.setPosition(ARM_DOWN_POSITION);
        armLeft.setPosition(ARM_DOWN_POSITION);
    }

    public void armRest() {
        armRight.setPosition(ARM_REST_POSITION);
        armLeft.setPosition(ARM_REST_POSITION);
    }

    public void wristUp() {
        wrist.setPosition(WRIST_UP_POSITION);
    }

    public void wristDown() {
        wrist.setPosition(WRIST_DOWN_POSITION);
    }

    // Multithreaded grab logic
    public void startGrab() {
        grabRunning = true;

        grabThread = new Thread(() -> {
            int state = 1;
            while (grabRunning && !Thread.currentThread().isInterrupted()) {
                state = grab(state);
                if (state == 0) {
                    grabRunning = false; // Stop when grab sequence is complete
                }
                try {
                    Thread.sleep(50); // Small delay to prevent excessive CPU usage
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        });
        grabThread.start();
    }

    public void stopGrab() {
        grabRunning = false;
        if (grabThread != null) {
            grabThread.interrupt();
        }
    }

    private int grab(int state) {
        switch (state) {
            case 1:
                clawOpen();
                wristDown();
                timer.reset();
                state = 2;
                break;
            case 2:
                if (timer.milliseconds() > 300) state = 3;
                break;
            case 3:
                armDown();
                timer.reset();
                state = 4;
                break;
            case 4:
                if (timer.milliseconds() > 250) state = 5;
                break;
            case 5:
                clawClose();
                timer.reset();
                state = 6;
                break;
            case 6:
                if (timer.milliseconds() > 200) state = 0; // Grab sequence complete
                break;
            default:
                break;
        }
        return state;
    }

    public void grabUp() {
        new Thread(() -> {
            armRest();
            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            wristUp();
        }).start();
    }
}
