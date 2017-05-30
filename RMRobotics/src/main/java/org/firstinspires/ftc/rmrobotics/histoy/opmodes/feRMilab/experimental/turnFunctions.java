package org.firstinspires.ftc.rmrobotics.histoy.opmodes.feRMilab.experimental;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.core.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.enums.Color;
import org.firstinspires.ftc.rmrobotics.util.enums.Direction;
import org.firstinspires.ftc.rmrobotics.util.enums.Drive;

/**
 * Created by Sue on 2/19/2017.
 */

@Autonomous(name = "Test Functions")
@Disabled
public class turnFunctions extends FeRMiLinear {

    @Override
    public void runOpMode() throws InterruptedException {
        super.initialize(Color.RED, DcMotor.RunMode.RUN_USING_ENCODER, Direction.FORWARD);

        /*telemetry.addData("1 Turn Function: turnRight", "90");
        telemetry.update();
        turn(Direction.RIGHT, 90, 0.4);
        setDrive(0);
        sleep(2000);

        telemetry.addData("1 Turn Function: turnRight", "0");
        telemetry.update();
        turn(Direction.RIGHT, 0, 0.4);
        setDrive(0);
        sleep(2000);

        telemetry.addData("1 Turn Function: turnLeft", "90");
        telemetry.update();
        turn(Direction.LEFT, 90, 0.4);
        setDrive(0);
        sleep(2000);

        telemetry.addData("1 Turn Function: turnLeft", "0");
        telemetry.update();
        turn(Direction.LEFT, 0, 0.4);
        setDrive(0);
        sleep(2000);

        telemetry.addData("1 Turn Function: turnCenter", "90");
        telemetry.update();
        turn(Direction.CENTER, 90, 0.4);
        setDrive(0);
        sleep(2000);

        telemetry.addData("1 Turn Function: turnCenter", "0");
        telemetry.update();
        turn(Direction.CENTER, 0, 0.4);
        setDrive(0);
        sleep(2000);*/

        telemetry.addData("1 Drive Function: Drive Time", "1 Second");
        telemetry.update();
        drive(Drive.TIME, 1000, 0.4);
        setDrive(0);
        sleep(2000);

        telemetry.addData("1 Drive Function: Drive Encoder", "1000");
        telemetry.update();
        drive(Drive.ENCODER, 1000, 0.4);
        setDrive(0);
        sleep(2000);
        telemetry.addData("1 Drive Function: Drive Encoder", "-1000");
        telemetry.update();
        drive(Drive.ENCODER, -1000, 0.4);
        setDrive(0);
        sleep(2000);


        telemetry.addData("1 Drive Function: Drive Range", "20");
        telemetry.update();
        drive(Drive.TIME, 20, 0.4);
        setDrive(0);
        sleep(2000);

        telemetry.addData("1 Drive Function: Drive Range", "30");
        telemetry.update();
        drive(Drive.TIME, 30, 0.4);
        setDrive(0);
        sleep(2000);




    }

}
