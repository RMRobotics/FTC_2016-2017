package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.core.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.enums.Color;
import org.firstinspires.ftc.rmrobotics.util.enums.Direction;
import org.firstinspires.ftc.rmrobotics.util.enums.Drive;

/**
 * Created by RM Robotics on 2/4/2017.
 */

@Autonomous(name = "RED: Cap")
public class Cap extends FeRMiLinear {

    @Override
    public void runOpMode() throws InterruptedException {
        super.initialize(Color.NEITHER, DcMotor.RunMode.RUN_USING_ENCODER, Direction.FORWARD);

        //sleep for 10 seconds
        // sleep(10000);

        //drive forward
        drive(Drive.ENCODER, -3000, 0.3);

        //wait
        sleep(3000);

        //drive forward
        drive(Drive.ENCODER, -500, 0.2);

        //knock-off capball
        //turnRobot(500);

        while (opModeIsActive()){
            addTelemetry();
        }
    }

}
