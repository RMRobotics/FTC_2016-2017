package org.firstinspires.ftc.rmrobotics.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.rmrobotics.util.MotorType;

/**
 * Created by Simon on 9/3/2016.
 */

public class motor {
    private DcMotor parent;
    private DcMotor.Direction defDir;
    private DcMotor.Direction opDir;
    private DcMotor.RunMode rmode;
    private DcMotor.ZeroPowerBehavior zmode;
    private MotorType type;

    private double desPower;

    public motor(DcMotor dc, DcMotor.Direction d, DcMotor.RunMode rm, DcMotor.ZeroPowerBehavior zm, MotorType t) {
        parent = dc;
        defDir = d;
        rmode = rm;
        zmode = zm;
        type = t;

        parent.setDirection(defDir);
        parent.setMode(rmode);
        parent.setZeroPowerBehavior(zmode);

        if (d == DcMotorSimple.Direction.FORWARD) {
            opDir = DcMotorSimple.Direction.REVERSE;
        } else {
            opDir = DcMotorSimple.Direction.FORWARD;
        }
    }

    public void setPower(double p) {
        desPower = p;
    }

    public void setCurrentPower() {
        double absDesPower = Math.abs(desPower);
        dirCheck(desPower);
        if (rmode == DcMotor.RunMode.RUN_TO_POSITION) {
            if (inRange(parent.getCurrentPosition(), parent.getTargetPosition(), 10)) {
                absDesPower = 0;
            }
        }
        if (absDesPower != 0.0) {
            absDesPower = Range.clip(absDesPower, 0.1, 1.0);
        }
        parent.setPower(absDesPower);
    }

    public double getPower() {
        return parent.getPower();
    }

    public void setTarPos(double r) {
        parent.setTargetPosition(rotToEnc(r));
    }

    public double getCurPos() {
        if (parent.getDirection() == DcMotorSimple.Direction.FORWARD) {
            return parent.getCurrentPosition();
        } else {
            return -parent.getCurrentPosition();
        }
    }

    public int getTarPos() {
        if (parent.getDirection() == DcMotorSimple.Direction.FORWARD) {
            return parent.getTargetPosition();
        } else {
            return -parent.getTargetPosition();
        }
    }

    public void setMode(DcMotor.RunMode m) {
        parent.setMode(m);
    }

    public DcMotor.RunMode getMode() {
        return parent.getMode();
    }

    public DcMotor.RunMode getRMode() {
        return rmode;
    }

    public DcMotor getParent() {
        return this.parent;
    }

    private void dirCheck(double d) {
        if (d < 0) {
            parent.setDirection(opDir);
        } else {
            parent.setDirection(defDir);
        }
    }

    private boolean inRange(double x, double y, double r) {
        if (x < y + Math.abs(r) && x > y - Math.abs(r)) {
            return true;
        }
        return false;
    }

    private int rotToEnc(double r) {
        return (int) (r*mPulse());
    } //converts rotation value to encoder value

    private double mPulse() {
        int enc;
        switch (type) {
            case NVRST20:
                enc = 560;
                break;
            case NVRST40:
                enc = 1120;
                break;
            case NVRST60:
                enc = 1680;
                break;
            default:
                enc = 1440;
                break;
        }
        return enc;
    }
}