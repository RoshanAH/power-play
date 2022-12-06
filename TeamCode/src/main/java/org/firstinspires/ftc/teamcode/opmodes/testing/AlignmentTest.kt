package org.firstinspires.ftc.teamcode.opmodes.testing

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs
import kotlin.math.sign
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.components.Mecanum
import org.firstinspires.ftc.teamcode.components.Webcam
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.core.Robot

@TeleOp(group = "testing")
@Config
class AlignmentTest : BaseOpmode() {

  companion object {
    @JvmField var kp = 0.0
    @JvmField var alignment = Webcam.Alignment.NONE
  }

  lateinit var camera: Webcam
  lateinit var dt: Mecanum

  override fun setRobot() =
      object : Robot() {
        override fun mapHardware(map: HardwareMap) {
          camera =
              Webcam(map, "camera", "mount") { dt.relativeVel.y }.apply {
                startCamera()
                webcam.setPipeline(alignmentPipeline)
              }

          dt = Mecanum(map, "fl", "fr", "bl", "br").apply{
            ticksPerInch = 30.9861111
            ticksPerDegree = 4.98611
          }
          addComponents(dt, camera)
        }
      }

  override fun onInit(scope: CoroutineScope) {
    val dash = FtcDashboard.getInstance()
    telemetry = MultipleTelemetry(telemetry, dash.getTelemetry())
    dash.startCameraStream(camera.webcam, 30.0)
  }

  override fun onStart(scope: CoroutineScope) = runBlocking {
    while (!camera.cameraRunning) yield()
  }

  override fun onUpdate(scope: CoroutineScope) {
    camera.kp = kp 
    camera.alignment = alignment

    dt.drive(gamepad1)

    telemetry.addData("objects found", camera.objects.size)
    telemetry.addData("forward vel", dt.relativeVel.y)
    telemetry.addData("closest", camera.closest?.xy)
    telemetry.addData("phi", camera.closest?.phi?.deg)
    telemetry.update()
  }
}
