package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.utils.GamepadListener;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public abstract class BaseOpmode extends LinearOpMode {
  private Robot robot;

  protected GamepadListener gamepadListener1 = new GamepadListener();
  protected GamepadListener gamepadListener2 = new GamepadListener();
  @Override
  public void runOpMode() throws InterruptedException {
    robot = setRobot();


    try {
      robot.mapHardware(hardwareMap);
      robot.components.forEach(Component::init);
    }catch (NullPointerException e){
      StringWriter sw = new StringWriter();
      e.printStackTrace(new PrintWriter(sw));
      telemetry.addLine(sw.toString());
      telemetry.update();

      waitForStart();
      return;
    }

    onInit();

    waitForStart();

    onStart();
    robot.components.forEach(Component::start);



    while(opModeIsActive()){
      gamepadListener1.update(gamepad1);
      gamepadListener2.update(gamepad2);
      onUpdate();
      robot.components.forEach(Component::update);
    }
  }
  protected double getBatteryVoltage() {
    double result = Double.POSITIVE_INFINITY;
    for (VoltageSensor sensor : hardwareMap.voltageSensor) {
      double voltage = sensor.getVoltage();
      if (voltage > 0) {
        result = Math.min(result, voltage);
      }
    }
    return result;
  }
  protected abstract Robot setRobot();

  public void onInit() {}
  public void onStart() {}
  public void onUpdate() {}

}
