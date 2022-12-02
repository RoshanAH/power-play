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
    @JvmField var alignment = Webcam.Alignment.NONE
    @JvmField var followDist = 5.0
  }

  lateinit var camera: Webcam
  lateinit var dt: Mecanum

  override fun setRobot() =
      object : Robot() {
        override fun mapHardware(map: HardwareMap) {
          camera =
              Webcam(map, "camera", "mount") { (dt.vel.fl + dt.vel.fr + dt.vel.bl + dt.vel.br) * 0.25 }.apply {
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
    camera.kp = kCam
    camera.alignment = alignment

    val closest = camera.closest

    if(closest != null){
      dt.drive(0.0, closest.xy.y * ky, closest.theta.rad * kw)
    }else{
      dt.move(0.0, 0.0, 0.0, 0.0)
    }

    telemetry.addData("objects found", camera.objects.size)
    telemetry.addData("closest", camera.closest?.xy)
    telemetry.update()
  }
}
