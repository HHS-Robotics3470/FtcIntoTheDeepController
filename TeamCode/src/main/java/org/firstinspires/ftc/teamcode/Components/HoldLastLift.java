package org.firstinspires.ftc.teamcode.Components;

public class HoldLastLift {
    private static int lastLiftHeight = 0;

    public static void setHeight(int height)
    {
        lastLiftHeight = height;
    }

    public static int getHeight()
    {
        return lastLiftHeight;
    }
}
