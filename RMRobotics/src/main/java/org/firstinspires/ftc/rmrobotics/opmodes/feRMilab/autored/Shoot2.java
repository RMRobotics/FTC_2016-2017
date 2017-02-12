package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.Color;

/**
 * Created by Simon on 2/12/17.
 */

@Autonomous(name = "BOTH: Shoot2")
public class Shoot2 extends FeRMiLinear {
    @Override
    public void runOpMode() throws InterruptedException {
        super.initialize(Color.RED, DcMotor.RunMode.RUN_USING_ENCODER);

        setDrive(0.4);
        sleep(500);
        setDrive(0);
        flyL.setPower(0.985);
        flyR.setPower(0.985);
        sleep(200);
        index.setPosition(0.5);
        sleep(200);
        belt.setPower(1.0);
        sleep(1000);

        flyL.setPower(0);
        flyR.setPower(0);
        belt.setPower(0);
        stop();
    }
}
