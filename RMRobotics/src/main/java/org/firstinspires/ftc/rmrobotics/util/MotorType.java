package org.firstinspires.ftc.rmrobotics.util;

/**
 * Created by Simon on 2/8/2016.
 */

public enum MotorType {
    TETRIX(0),
    NVRST20(20),
    NVRST40(40),
    NVRST60(60);

    private int motorTypeName;

    MotorType(int t) {
        this.motorTypeName = t;
    }

    public String toString() {
        return Integer.toString(motorTypeName);
    }

    public static MotorType toType(long t) {
        MotorType[] types = values();
        for (int i = 0; i < types.length; i++) {
            if (types[i].motorTypeName == t) {
                return types[i];
            }
        }
        return null;
    }
}