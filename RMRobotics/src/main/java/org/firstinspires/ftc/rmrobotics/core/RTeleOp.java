package org.firstinspires.ftc.rmrobotics.core;

public abstract class RTeleOp extends ROpMode {

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void updateInput(){
        control.update(gamepad1, gamepad2);
    }

}

