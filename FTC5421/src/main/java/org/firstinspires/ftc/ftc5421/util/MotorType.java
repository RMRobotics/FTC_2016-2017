package org.firstinspires.ftc.ftc5421.util;

/**
 * Created by Simon on 2/8/2016.
 */

public enum MotorType {
    TETRIX(0),
    NVRST_20(20),
    NVRST_40(40),
    NVRST_60(60);

    private int motorTypeName;

    MotorType(int t) {
        this.motorTypeName = t;
    }

    public String toString() {
        return Integer.toString(motorTypeName);
    }

    public static MotorType toType(int t) {
        MotorType[] types = values();
        for (int i = 0; i < types.length; i++) {
            if (types[i].motorTypeName == t) {
                return types[i];
            }
        }
        return null;
    }
}