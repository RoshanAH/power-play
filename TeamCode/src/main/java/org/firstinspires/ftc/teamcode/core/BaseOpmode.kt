package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.Arrays;
import java.util.HashMap;
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.CoroutineScope

abstract class BaseOpmode : LinearOpMode() {
  private lateinit var robot: Robot

  val gamepadListener1 = GamepadListener()
  val gamepadListener2 = GamepadListener()
  override fun runOpMode() = runBlocking {   
    robot = setRobot();

    try {
      robot.mapHardware(hardwareMap)
      robot.components.forEach { it.init(this) }
    }catch (e: NullPointerException){
      val sw = StringWriter()
      e.printStackTrace(PrintWriter(sw))
      telemetry.addLine(sw.toString())
      telemetry.update()

      waitForStart()
      return@runBlocking
    }

    onInit(this)

    waitForStart()

    onStart(this)
    robot.components.forEach { it.start(this) }

    while(opModeIsActive()){
      gamepadListener1.update(gamepad1)
      gamepadListener2.update(gamepad2)
      onUpdate(this)
      robot.components.forEach { it.update(this) }
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

  open fun onInit(scope: CoroutineScope) {}
  open fun onStart(scope: CoroutineScope) {}
  open fun onUpdate(scope: CoroutineScope) {}
}
