package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.core.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.enums.Color;
import org.firstinspires.ftc.rmrobotics.util.enums.Direction;

import static org.firstinspires.ftc.rmrobotics.util.enums.Drive.TIME;

/**
 * Created by Simon on 2/12/17.
 */

@Autonomous(name = "BOTH: Shoot2")
public class Shoot2 extends FeRMiLinear {
    @Override
    public void runOpMode() throws InterruptedException {
        super.initialize(Color.RED, DcMotor.RunMode.RUN_USING_ENCODER, Direction.FORWARD);

        //sleep for 10 seconds
        // sleep(10000);

        // drive for .5 seconds at .4 voltage
        drive(TIME, 500, 0.4);
        setDrive(0);

        // turn on flywheels for .2 seconds at .975 seconds
        flyL.setPower(0.975);
        flyR.setPower(0.975);
        sleep(200);

        // open indexer
        index.setPosition(0.5);
        sleep(200);

        // turn on belt
        belt.setPower(1.0);
        sleep(1000);

        stop();
    }
}
