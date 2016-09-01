package org.firstinspires.ftc.ftc5421.hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Simon on 11/23/2015.
 */

public class RServo {

    private static final double MAX_POSITION = 1.0;
    private static final double MIN_POSITION = 0.01;

    private Servo parent;
    private Servo.Direction defaultDirection;
    private double minPosition;
    private double maxPosition;
    private double desiredPosition;
    private double currentPosition;
    private double initPos;

    public RServo(Servo s, Servo.Direction x, double min, double max, double init){
        parent = s;
        defaultDirection = x;
        minPosition = min;
        maxPosition = max;
        initPos = init;
        desiredPosition = init;
        parent.setDirection(defaultDirection);
    }

    public void setDesiredPosition(double d){
        desiredPosition = d;
    }

    public void updateCurrentPosition(){
        desiredPosition = Range.clip(desiredPosition, minPosition, maxPosition);
        currentPosition = desiredPosition;
    }

    public void setPosition(){
        parent.setPosition(currentPosition);
    }

    public double getPosition() {
        return parent.getPosition();
    }

    public void setInitPos() {parent.setPosition(initPos);}

}