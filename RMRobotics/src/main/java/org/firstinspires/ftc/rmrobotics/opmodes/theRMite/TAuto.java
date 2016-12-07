package org.firstinspires.ftc.rmrobotics.opmodes.theRMite;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.rmrobotics.core.RAuto;
import org.firstinspires.ftc.rmrobotics.core.state.State;
import org.firstinspires.ftc.rmrobotics.core.state.StateFactory;
import org.firstinspires.ftc.rmrobotics.util.Robot;
import org.firstinspires.ftc.rmrobotics.util.config.TheRMite;
import org.firstinspires.ftc.rmrobotics.util.StateNames;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Created by Kchen on 12/1/2016.
 */

@Autonomous(name = "TAuto")
@Disabled
public class TAuto extends RAuto{
    private TheRMite config;
    public int currentState = 0;
    public boolean stateInit = false;

    Map <StateNames,State> stateMap = new HashMap<>();

    List<StateNames> stateOrder = new ArrayList();

    public void init(){
        stateOrder.add(0, StateNames.init);
        stateOrder.add(1, StateNames.servoinit);
        stateOrder.add(2, StateNames.timer);

        for (int i=0;i==stateOrder.size();i++){
            StateFactory.addStates(stateMap, stateOrder.get(i));
        }
    }

    public void runState(){
        if (!stateInit){
            stateMap.get(stateOrder.get(currentState)).init();
            stateInit = true;
        }
        if(stateMap.get(stateOrder.get(currentState)).isDone()){
            stateInit = false;
            currentState++;
        }
    }

    @Override
    protected Robot setRobot() {
        return config = new TheRMite(hardwareMap);
    }

    @Override
    protected void addTelemetry() {
    }
}

