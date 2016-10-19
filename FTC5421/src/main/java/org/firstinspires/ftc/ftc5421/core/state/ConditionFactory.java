package org.firstinspires.ftc.ftc5421.core.state;

import org.firstinspires.ftc.ftc5421.hardware.motor;

import java.util.Map;

/**
 * Created by michaelblob on 10/19/16.
 */

public class ConditionFactory {

    public static Condition time(final long t) {
        return new Condition() {
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

    public static Condition encoder(final Map<motor, Double> map) {
        return new Condition() {
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

    public static Condition servoPos() {
        return new Condition() {
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
}
