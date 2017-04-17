package org.firstinspires.ftc.rmrobotics.histoy.opmodes.feRMilab;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.rmrobotics.histoy.library.RAuto;
import org.firstinspires.ftc.rmrobotics.util.config.Robot;
import org.firstinspires.ftc.rmrobotics.util.config.FeRMilab;
import org.firstinspires.ftc.rmrobotics.util.enums.Op;

/**
 * Created by Simon on 11/30/2016.
 */

@Autonomous(name = "FAuto")
@Disabled
public class FAuto extends RAuto {

    private FeRMilab config;

    @Override
    protected Robot setRobot() {
        return config = new FeRMilab(hardwareMap, Op.AUTON);
    }

    @Override
    protected void setAlliance() {

    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
