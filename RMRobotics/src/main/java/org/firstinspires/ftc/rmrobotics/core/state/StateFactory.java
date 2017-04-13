package org.firstinspires.ftc.rmrobotics.core.state;

import org.firstinspires.ftc.rmrobotics.util.StateNames;

import java.util.Map;

/**
 * Created by Simon on 11/20/16.
 */
public class StateFactory {
    public static void addStates(Map<StateNames, State> StateNames, StateNames name){
        if(name == org.firstinspires.ftc.rmrobotics.util.StateNames.init) {
            StateNames.put(org.firstinspires.ftc.rmrobotics.util.StateNames.init, States.timed(5));
        }
    }
}
