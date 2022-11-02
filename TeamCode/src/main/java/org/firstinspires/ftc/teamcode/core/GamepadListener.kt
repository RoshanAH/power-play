package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.core.Component;

class GamepadListener{

    val a = Button();
    val b = Button();
    val x = Button();
    val y = Button();

    val du = Button();
    val dd = Button();
    val dl = Button();
    val dr = Button();

    val rb = Button();
    val lb = Button();

    val lsb = Button();
    val rsb = Button();

    var plx = 0.0;
    var ply = 0.0;
    var prx = 0.0;
    var pry = 0.0;

    var plt = 0.0;
    var prt = 0.0;

    var onJoystickMove: () -> Unit = {}
    var onRTMove: () -> Unit = {}
    var onLTMove: () -> Unit = {}

    private val buttons = listOf(
            a,
            b,
            x,
            y,
            du,
            dd,
            dl,
            dr,
            rb,
            lb,
            lsb,
            rsb
    )

    fun update(gamepad: Gamepad) {
        a.update(gamepad.a);
        b.update(gamepad.b);
        x.update(gamepad.x);
        y.update(gamepad.y);

        du.update(gamepad.dpad_up);
        dd.update(gamepad.dpad_down);
        dl.update(gamepad.dpad_left);
        dr.update(gamepad.dpad_right);

        rb.update(gamepad.right_bumper);
        lb.update(gamepad.left_bumper);

        lsb.update(gamepad.left_stick_button);
        rsb.update(gamepad.right_stick_button);

        if (gamepad.left_stick_x.toDouble() != plx ||
                gamepad.left_stick_y.toDouble() != ply ||
                gamepad.right_stick_x.toDouble() != prx ||
                gamepad.right_stick_y.toDouble() != pry) {

            plx = gamepad.left_stick_x.toDouble();
            ply = gamepad.left_stick_y.toDouble();
            prx = gamepad.right_stick_x.toDouble();
            pry = gamepad.right_stick_y.toDouble();

            onJoystickMove()
        }

        if(gamepad.left_trigger.toDouble() != plt){
            plt = gamepad.left_trigger.toDouble();
            onLTMove()
        }

        if(gamepad.right_trigger.toDouble() != prt){
            prt = gamepad.right_trigger.toDouble();
            onRTMove()
        }
    }

}

    class Button {
        var onPress: () -> Unit = {}
        var onRelease: () -> Unit = {}
        var onHold: () -> Unit = {}
        var isPressed = false;
        var isReleased = false;
        var isHeld = false;

        var pressBuffer = false;
        var releaseBuffer = false;

        fun update(buttonState: Boolean) {
            if (buttonState && !pressBuffer) {
                isPressed = true;
                pressBuffer = true;
                onPress()
            } else {
                if (!buttonState) {
                    pressBuffer = false;
                }
                isPressed = false;
            }

            if (!buttonState && !releaseBuffer) {
                isReleased = true;
                releaseBuffer = true;
                onRelease()
            } else {
                if (buttonState) {
                    releaseBuffer = false;
                }
                isReleased = false;
            }

            isHeld = buttonState;

            if (buttonState) {
                onHold()
            }
        }
    }
