package org.firstinspires.ftc.rmrobotics.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Simon on 9/4/2016.
 */

public class crservo {
    private CRServo parent;
    private CRServo.Direction defDir;
    private CRServo.Direction opDir;

    private double desPower;

    public crservo(CRServo s, CRServo.Direction d) {
        parent = s;
        defDir = d;

        if (d == CRServo.Direction.FORWARD) {
            opDir = CRServo.Direction.REVERSE;
        } else {
            opDir = CRServo.Direction.FORWARD;
        }
    }

    public void setPower(double p) {
        desPower = p;
    }

    public double getPower() {
        return parent.getPower();
    }

    public CRServo getParent() {
        return this.parent;
    }

    public void setCurrentPower() {
        double absDesPower = Math.abs(desPower);
        dirCheck(desPower);
        if (absDesPower != 0.0) {
            absDesPower = Range.clip(absDesPower, 0.1, 1.0);
        }
        parent.setPower(absDesPower);
    }

    private void dirCheck(double d) {
        if (d < 0) {
            parent.setDirection(opDir);
        } else {
            parent.setDirection(defDir);
        }
    }
}