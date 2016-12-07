package org.firstinspires.ftc.rmrobotics.core.state;

import org.firstinspires.ftc.rmrobotics.hardware.motor;

import java.util.Map;

/**
 * Created by Simon on 11/29/16.
 */

public class States {
    public static State timed(final long t) {
        return new State() {
            private long time;

            @Override
            public void init() {
                time = System.currentTimeMillis() + t;
            }

            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= time;
            }
        };
    }

    public static State encoder(final Map<motor, Double> map) {
        return new State() {
            @Override
            public void init() {
                for (motor m : map.keySet()) {
                    m.setTarPos(map.get(m));
                }
            }

            @Override
            public boolean isDone() {
                boolean all = true;
                for (motor m : map.keySet()) {
                    if (Math.abs(m.getCurPos() - m.getTarPos()) > 10) {
                        all = false;
                    }
                }
                return all;
            }

        };
    }

    public static State servoPos() {
        return new State() {
            private long time;

            @Override
            public void init() {
                time = System.currentTimeMillis() + 500;
            }

            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= time;
            }

        };
    }

    public static State beaconColor(final int colorValue,final int beaconcolor){
        return new State(){
            private int red = 156;
            private int blue = 200;
            private int checkColor;

            @Override
            public void init(){
                if (beaconcolor == red){
                    checkColor = red;
                }else{
                    checkColor = blue;
                }
            }

            public boolean check() {
                if (colorValue == checkColor){
                    return true;
                }else{
                    return false;
                }
            }

            public boolean isDone(){
                return true;
            }
        };
    }
}
