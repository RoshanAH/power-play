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
    @JvmField var kw = 0.0
    @JvmField var kx = 0.0
    @JvmField var ky = 0.01
    @JvmField var kCam = 0.01
    @JvmField var aligning = false
    @JvmField var followDist = 5.0
  }

  lateinit var camera: Webcam
  lateinit var dt: Mecanum

  override fun setRobot() =
      object : Robot() {
        override fun mapHardware(map: HardwareMap) {
          camera =
              Webcam(map, "camera", "mount") { (dt.flv + dt.frv + dt.blv + dt.brv) * 0.25 }.apply {
                startCamera()
                webcam.setPipeline(alignmentPipeline)
              }

          dt = Mecanum(map, "fl", "fr", "bl", "br")
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
    camera.kp = kCam
    camera.aligning = aligning
    telemetry.addData("objects found", camera.alignmentPipeline.objects.size)
    val center = camera.alignmentPipeline.center
    if (center != null && aligning) {
      val vertAlignment = (160.0 - abs(center.x)) / 160.0
      val forwardVel = (camera.dist - followDist) * ky * vertAlignment
      dt.drive(
          center.x * kx,
          abs(forwardVel).coerceIn(0.1 * vertAlignment, 0.3) * sign(forwardVel),
          center.x * kw * camera.dist
      )
      telemetry.addData("center", center.toString())
      telemetry.addData("measuredDistance", camera.dist)
    } else {
      dt.drive(0.0, 0.0, 0.0)
    }

    telemetry.update()
  }
}
