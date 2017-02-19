package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.Color;
import org.firstinspires.ftc.rmrobotics.util.Direction;

/**
 * Created by RM Robotics on 2/2/2017.
 */
@Autonomous(name = "RED: Ramp")
public class Ramp extends FeRMiLinear {

    @Override
    public void runOpMode() throws InterruptedException {
        super.initialize(Color.RED, DcMotor.RunMode.RUN_USING_ENCODER, Direction.BACKWARD);

        //turn at an angle
        turnCorner(45, 0.4, Direction.RIGHT);

        //drive forward
        driveEncoder(1200, -.4);

        //face ramp
        turnCorner(130, 0.4, Direction.RIGHT);

        //drive onto ramp
        driveEncoder(1500, -.3);

        while (opModeIsActive()){
            addTelemetry();
        }
    }
}
