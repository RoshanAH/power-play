package org.firstinspires.ftc.teamcode.utils;

public class Timer {
    public long startTime;

    public void resetTimer(){
        startTime = System.nanoTime();
    }

    public double getElapsedTime(){
        return (System.nanoTime() - startTime)/1E9d;
    }
}
