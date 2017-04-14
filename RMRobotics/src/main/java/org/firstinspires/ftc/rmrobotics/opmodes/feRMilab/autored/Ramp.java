package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.core.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.enums.Color;
import org.firstinspires.ftc.rmrobotics.util.enums.Direction;
import org.firstinspires.ftc.rmrobotics.util.enums.Drive;

import static org.firstinspires.ftc.rmrobotics.util.enums.Direction.RIGHT;

/**
 * Created by RM Robotics on 2/2/2017.
 */

@Autonomous(name = "RED: Ramp")
public class Ramp extends FeRMiLinear {

    @Override
    public void runOpMode() throws InterruptedException {
        super.initialize(Color.RED, DcMotor.RunMode.RUN_USING_ENCODER, Direction.BACKWARD);

        //sleep for ten seconds
        // sleep(10000);

        //turn at an angle
        turn(RIGHT, 45, 0.4);

        //drive forward
        drive(Drive.ENCODER, -1200, 0.4);

        //face ramp
        turn(RIGHT, 130, 0.4);

        //drive onto ramp
        drive(Drive.ENCODER, -1500, 0.3);

        while (opModeIsActive()){
            addTelemetry();
        }
    }
}
