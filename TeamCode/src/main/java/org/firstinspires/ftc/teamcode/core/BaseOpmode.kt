package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.Arrays;
import java.util.HashMap;

abstract class BaseOpmode : LinearOpMode() {
  private lateinit var robot: Robot

  val gamepadListener1 = GamepadListener()
  val gamepadListener2 = GamepadListener()
  override fun runOpMode() {   
    robot = setRobot();

    try {
      robot.mapHardware(hardwareMap)
      robot.components.forEach { it.init() }
    }catch (e: NullPointerException){
      val sw = StringWriter()
      e.printStackTrace(PrintWriter(sw))
      telemetry.addLine(sw.toString())
      telemetry.update()

      waitForStart()
      return
    }

    onInit()

    waitForStart()

    onStart()
    robot.components.forEach { it.start() }

    while(opModeIsActive()){
      gamepadListener1.update(gamepad1)
      gamepadListener2.update(gamepad2)
      onUpdate()
      robot.components.forEach { it.update() }
    }
  }



  protected fun getBatteryVoltage(): Double {
    var result = Double.POSITIVE_INFINITY;
    for (sensor in hardwareMap.voltageSensor) {
      val voltage = sensor.getVoltage();
      if (voltage > 0) {
        result = Math.min(result, voltage);
      }
    }
    return result
  }

  protected abstract fun setRobot(): Robot

  open fun onInit() {}
  open fun onStart() {}
  open fun onUpdate() {}
}
