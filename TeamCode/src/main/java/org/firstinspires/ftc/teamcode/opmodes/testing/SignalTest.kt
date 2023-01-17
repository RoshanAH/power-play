package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.config.Config
import com.roshanah.jerky.math.deg
import org.firstinspires.ftc.teamcode.components.Webcam
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield

@TeleOp(group = "testing")
@Config
class SignalTest : LinearOpMode(){

  companion object {
    @JvmField var cameraAngle = 0.0
  }

  override fun runOpMode() = runBlocking{
    val camera = Webcam(hardwareMap, "camera", "mount", { 0.0 }).apply{
      startCamera()
    }

    val dash = FtcDashboard.getInstance()
    telemetry = MultipleTelemetry(telemetry, dash.getTelemetry())
    dash.startCameraStream(camera.webcam, 30.0)

    waitForStart()

    while(!camera.cameraRunning) yield()
    camera.webcam.setPipeline(camera.signalPipeline)

    var lastTime = System.nanoTime() * 1e-9
    while (opModeIsActive()){
      val time = System.nanoTime() * 1e-9
      val deltaTime = time - lastTime
      lastTime = time

      camera.phi = cameraAngle.deg 
      camera.update(this)

      telemetry.addData("signal", camera.signalPipeline.signalVal)
      telemetry.addData("image quality", dash.getImageQuality())
      telemetry.addData("mountPos", camera.mount.getPosition())
      telemetry.addData("dt", deltaTime)
      telemetry.update()
    }
  }
}
