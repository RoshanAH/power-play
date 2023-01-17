package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ReadWriteFile
import com.qualcomm.hardware.lynx.LynxModule
import com.roshanah.jerky.math.Pose

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.Arrays;
import java.util.HashMap;
import kotlinx.coroutines.*
import org.firstinspires.ftc.robotcore.internal.system.AppUtil

abstract class BaseOpmode : LinearOpMode() {
  private lateinit var robot: Robot
  private var hubs = listOf<LynxModule>()

  val gamepadListener1 = GamepadListener()
  val gamepadListener2 = GamepadListener()
  val scope = CoroutineScope(newSingleThreadContext("main"))

  override fun runOpMode() {

    hubs = hardwareMap.getAll(LynxModule::class.java)
    hubs.forEach { it.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)}
    robot = setRobot();

    try {
      robot.mapHardware(hardwareMap)
      robot.components.forEach { it.init(scope) }
    }catch (e: NullPointerException){
      val sw = StringWriter()
      e.printStackTrace(PrintWriter(sw))
      telemetry.addLine(sw.toString())
      telemetry.update()

      waitForStart()
      return
    }

    onInit(scope)

    waitForStart()

    onStart(scope)
    robot.components.forEach { it.start(scope) }

    while(opModeIsActive()){
      gamepadListener1.update(gamepad1)
      gamepadListener2.update(gamepad2)
      onUpdate(scope)
      robot.components.forEach { it.update(scope) }
      clearBulkCache()
    }

    onStop(scope)
    scope.cancel()
  }
  
  fun clearBulkCache() = hubs.forEach { it.clearBulkCache() }

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
  open fun onStop(scope: CoroutineScope) {}
}


val Gamepad.pose: Pose
  get() = Pose(
  left_stick_x.toDouble(), 
 -left_stick_y.toDouble(), 
 -right_stick_x.toDouble()
) 

