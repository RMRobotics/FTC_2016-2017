package org.firstinspires.ftc.rmrobotics.hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Simon on 9/4/2016.
 */

public class servo {
    private Servo parent;
    private Servo.Direction dir;
    private double minPos;
    private double maxPos;
    private double initPos;

    private double desPos;

    public servo(Servo s, Servo.Direction d, double min, double max, double init) {
        parent = s;
        dir = d;
        minPos = min;
        maxPos = max;
        initPos = init;
    }

    public void setPosition(double p) {
        desPos = p;
    }

    public void setCurPosition() {
        desPos = Range.clip(desPos, 0.0, 1.0);
        parent.setPosition(desPos);
    }

    public double getPosition() {
        return parent.getPosition();
    }

    public void setInitPos() {
        parent.setPosition(initPos);
    }

    public Servo getParent() {
        return this.parent;
    }
}

