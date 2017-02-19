package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.util.Color;
import org.firstinspires.ftc.rmrobotics.util.Direction;

/**
 * Created by poofs on 2/19/2017.
 */

@Autonomous(name = "Test Turn Functions")
public class turnFunctions extends FeRMiLinear{

    @Override
    public void runOpMode() throws InterruptedException {
        super.initialize(Color.RED, DcMotor.RunMode.RUN_USING_ENCODER, Direction.FORWARD);

        telemetry.addData("1 Turn Function: turnCenter", "90");
        telemetry.update();
        turnCenter(90, 0.4);
        sleep(5000);

        telemetry.addData("1 Turn Function: turnCorner", "90");
        telemetry.update();
        turnCorner(90, 0.4, Direction.RIGHT);
        sleep(5000);
    }

}
