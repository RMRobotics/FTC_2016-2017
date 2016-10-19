package org.firstinspires.ftc.ftc5421.util;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftc5421.control.Dpad;
import org.firstinspires.ftc.ftc5421.hardware.crservo;
import org.firstinspires.ftc.ftc5421.hardware.motor;
import org.firstinspires.ftc.ftc5421.hardware.servo;

import java.util.Map;

/**
 * Created by Simon on 10/18/2016.
 */

public class State {
    ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    boolean alreadyExec = false;

    StateType type;
    Map<motor, Double> motors;
    Map<servo, Double> servos;
    Map<crservo, Double> crservos;
    Map<HardwareDevice, Object> sensors;
    double time;

    public State(StateType t, Map<motor, Double> m, Map<servo, Double> s, Map<crservo, Double> cr, Map<HardwareDevice, Object> sn, double ti) { //GENERAL
        type = t;
        motors = m;
        servos = s;
        crservos = cr;
        sensors = sn;
        time = ti;
    }

}
