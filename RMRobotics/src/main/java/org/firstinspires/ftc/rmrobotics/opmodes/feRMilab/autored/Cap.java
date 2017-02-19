package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.Color;
import org.firstinspires.ftc.rmrobotics.util.Direction;

/**
 * Created by RM Robotics on 2/4/2017.
 */
@Autonomous(name = "RED: Cap")
public class Cap extends FeRMiLinear {

    @Override
    public void runOpMode() throws InterruptedException {
        super.initialize(Color.NEITHER, DcMotor.RunMode.RUN_USING_ENCODER, Direction.FORWARD);

        //drive forward
        driveEncoder(3000, -.3);

        //wait
        sleep(3000);

        //drive forward
        driveEncoder(500, -.2);

        //knock-off capball
        //turnRobot(500);

        while (opModeIsActive()){
            addTelemetry();
        }
    }

}
