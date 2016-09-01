package org.firstinspires.ftc.ftc5421.hardware;

/**
 * Created by Simon on 2/8/2016.
 */

public enum MotorType {
    TETRIX("Tetrix"),
    NVRST_20("NeveRest 20"),
    NVRST_40("NeveRest 40"),
    NVRST_60("NeveRest 60");

    String motorTypeName;

    MotorType(String stringD) {
        this.motorTypeName = stringD;
    }

    public String toString() {
        return motorTypeName;
    }
}