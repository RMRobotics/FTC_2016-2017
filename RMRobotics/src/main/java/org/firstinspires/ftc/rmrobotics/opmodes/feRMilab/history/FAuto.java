package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.history;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.rmrobotics.core.RAuto;
import org.firstinspires.ftc.rmrobotics.util.config.Robot;
import org.firstinspires.ftc.rmrobotics.util.config.FeRMilab;

/**
 * Created by Simon on 11/30/2016.
 */

@Autonomous(name = "FAuto")
@Disabled
public class FAuto extends RAuto {

    private FeRMilab config;

    @Override
    public void runState() {

    }

    @Override
    protected Robot setRobot() {
        return config = new FeRMilab(hardwareMap, false);
    }
}
