package org.firstinspires.ftc.ftc5421.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Math.abs;

public class RMotor {

    private static final double MAX_POWER = 1.0;
    private static final double MIN_POWER = 0.1;
    private static final double MAX_ACCEL = 0.03;
    private static final double TETRIX_ENC = 1440;
    private static final double NVRST20_ENC = 560;
    private static final double NVRST40_ENC = 1120;
    private static final double NVRST60_ENC = 1680;

    private DcMotor parent;
    private DcMotor.Direction defaultDirection;
    private double minPower;
    private double maxPower;
    private MotorType motorType;
    private double motorPulse;
    private double desiredPower;
    private double currentPower;
    private int curPos;
    private int tarPos;

    public RMotor(DcMotor dc, DcMotor.Direction d, double min, double max, MotorType mType) {
        parent = dc;
        defaultDirection = d;
        minPower = min;
        maxPower = max;
        motorType = mType;
        motorPulse = motorPulse();

    } //Todo add string to send in DbgLog confirming motor settings once configured

    public void setDesiredPower(double d) {
        desiredPower = d;
    }

    public void updateCurrentPower() {
        /**
         * let it be zero if it needs to be
         * absolute value and switch direction if needed
         * clip to be within range
         *
         */
        double absDesPower = abs(desiredPower);
        if (parent.getTargetPosition() == parent.getCurrentPosition() && parent.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            desiredPower = 0;
            currentPower = desiredPower;
        } else if (absDesPower == 0.0 || absDesPower < minPower) {
            currentPower = desiredPower;
        } else {
            if (desiredPower < 0.0) {
                if (defaultDirection == DcMotor.Direction.FORWARD) {
                    parent.setDirection(DcMotor.Direction.REVERSE);
                } else if (defaultDirection == DcMotor.Direction.REVERSE) {
                    parent.setDirection(DcMotor.Direction.FORWARD);
                }
            } else {
                if (defaultDirection == DcMotor.Direction.FORWARD) {
                    parent.setDirection(DcMotor.Direction.FORWARD);
                } else if (defaultDirection == DcMotor.Direction.REVERSE) {
                    parent.setDirection(DcMotor.Direction.REVERSE);
                }
            }
            desiredPower = absDesPower;
            if (desiredPower > maxPower) {
                desiredPower = maxPower;
            } else if (desiredPower < minPower) {
                desiredPower = 0;
            }
            currentPower = desiredPower;
        }
    }

    public void setCurrentPower() {
        if (desiredPower > currentPower) {
            currentPower += MAX_ACCEL;
            if (currentPower > maxPower) {
                currentPower = maxPower;
            }
            parent.setPower(currentPower);
        } else if (desiredPower < currentPower) {
            currentPower -= MAX_ACCEL;
            if (currentPower < minPower) {
                currentPower = 0;
            }
            parent.setPower(currentPower);
        } else {
            parent.setPower(currentPower);
        }
    }

    public void setEncoderMove(double rotation, double power) {
        curPos = parent.getCurrentPosition();
        tarPos = curPos + (int)(rotation * 1120);
        parent.setTargetPosition(tarPos);
        setDesiredPower(power);
    }

    private double motorPulse() {
        switch (motorType) {
            case NVRST_20:
                return NVRST20_ENC;
            case NVRST_40:
                return NVRST40_ENC;
            case NVRST_60:
                return NVRST60_ENC;
            default:
                return TETRIX_ENC;
        }
    }

    public double getCurrentPosition() {
        if (parent.getDirection() == DcMotor.Direction.FORWARD) {
            return parent.getCurrentPosition();
        } else {
            return -parent.getCurrentPosition();
        }
    }

    public int getTargetPosition() {
        if (parent.getDirection() == DcMotor.Direction.FORWARD) {
            return parent.getTargetPosition();
        } else {
            return -parent.getTargetPosition();
        }
    }

    public void setTargetPosition(int pos) {
        parent.setTargetPosition(pos);
    }

    public DcMotor.RunMode getMode() {
        return parent.getMode();
    }

    public void setMode(DcMotor.RunMode mode) {
        if (parent.getMode() != mode) {
            parent.setMode(mode);
        }
    }

    public double getPower() { return parent.getPower(); }

    public double getDesiredPower() { return desiredPower; }

    public String getMotorType() {
        return motorType.toString();
    }

    public void stop() { setDesiredPower(0); }

    public void setPowerFloat() {
        parent.setPowerFloat();
    }

}