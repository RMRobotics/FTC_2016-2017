package org.firstinspires.ftc.rmrobotics.control;

import com.qualcomm.robotcore.hardware.Gamepad;

public class RControl {

    private ControllerInput currentInput;
    private ControllerInput prevInput;

    public RControl(Gamepad g1, Gamepad g2){
        currentInput =  new ControllerInput(g1, g2);
        prevInput = currentInput;
    }

    public void update(Gamepad g1, Gamepad g2) {
        prevInput =  currentInput;
        currentInput =  new ControllerInput(g1, g2);
    }

    public boolean button(Controller c, Button b){
        return currentInput.button(c, b);
    }

    public boolean buttonPressed(Controller c, Button b){
        boolean prev = prevInput.button(c, b);
        boolean curr = currentInput.button(c, b);
        return curr && !prev;
    }

    public boolean buttonHeld(Controller c, Button b){
        boolean prev = prevInput.button(c, b);
        boolean curr = currentInput.button(c, b);
        return curr && prev;
    }

    public boolean buttonReleased(Controller c, Button b){
        boolean prev = prevInput.button(c, b);
        boolean curr = currentInput.button(c, b);
        return !curr && prev;
    }

    public double joystickValue(Controller c, Joystick j, Axis a){
        return currentInput.joystickValue(c, j, a);
    }

    public double triggerValue(Controller c, Trigger t){
        return currentInput.triggerValue(c, t);
    }

    public boolean dpadValue(Controller c, Dpad d) {
        return currentInput.dpad(c, d);
    }

}

class ControllerInput{

    private Gamepad game1;
    private Gamepad game2;

    public ControllerInput(Gamepad g1, Gamepad g2){
        game1 =  g1;
        game2 =  g2;
    }

    public double joystickValue(Controller c, Joystick j, Axis a){
        Gamepad g = getController(c);
        switch(j){
            case LEFT:
                switch(a){
                    case X:
                        return g.left_stick_x;
                    case Y:
                        return g.left_stick_y;
                }
            case RIGHT:
                switch(a){
                    case X:
                        return g.right_stick_x;
                    case Y:
                        return g.right_stick_y;
                }
        }
        return -1;
    }

    public double triggerValue(Controller c, Trigger t){
        Gamepad g = getController(c);
        switch (t){
            case LEFT:
                return g.left_trigger;
            case RIGHT:
                return g.right_trigger;
        }
        return -1;
    }

    public boolean button(Controller c, Button b){
        Gamepad g = getController(c);
        switch(b){
            case A:
                return g.a;
            case B:
                return g.b;
            case X:
                return g.x;
            case Y:
                return g.y;
            case LBUMP:
                return g.left_bumper;
            case RBUMP:
                return g.right_bumper;
            case BACK:
                return g.back;
            case START:
                return g.start;
            case LJOY:
                return g.left_stick_button;
            case RJOY:
                return g.right_stick_button;
        }
        return false;
    }

    public boolean dpad(Controller c, Dpad d) {
        Gamepad g = getController(c);
        switch (d) {
            case UP:
                return g.dpad_up;
            case DOWN:
                return g.dpad_down;
            case LEFT:
                return g.dpad_left;
            case RIGHT:
                return g.dpad_right;
        }
        return false;
    }

    private Gamepad getController(Controller c){
        switch (c){
            case ONE:
                return game1;
            case TWO:
                return game2;
        }
        return null; //Technically very bad practice...
    }

}