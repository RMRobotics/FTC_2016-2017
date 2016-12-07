package org.firstinspires.ftc.rmrobotics.core;

public abstract class RTeleOp extends ROpMode {

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop(){
        updateInput();
        calculate();
        updateHardware();
        addTelemetry();
    }

    protected abstract void calculate();

    public void updateInput(){
        control.update(gamepad1, gamepad2);
    }

}

