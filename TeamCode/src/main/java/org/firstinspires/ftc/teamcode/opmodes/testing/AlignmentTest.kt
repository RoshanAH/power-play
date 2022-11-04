package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.components.Webcam
import org.firstinspires.ftc.teamcode.components.Mecanum
import org.firstinspires.ftc.teamcode.core.GamepadListener
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.core.Robot
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import kotlinx.coroutines.CoroutineScope

@TeleOp(group = "testing")
@Config
class AlignmentTest : BaseOpmode(){

  companion object {
    @JvmField var kw = 0.0
    @JvmField var kx = 0.0
    @JvmField var ky = 0.0
    @JvmField var kCam = 0.5
    @JvmField var aligning = false
    @JvmField var followDist = 5.0
  }

  lateinit var camera: Webcam
  lateinit var dt: Mecanum

  override fun setRobot() = object : Robot(){
    override fun mapHardware(map: HardwareMap){
      camera = Webcam(map, "camera", "mount"){
        (dt.flv + dt.frv + dt.blv + dt.brv) * 0.25
      }.apply{
        startCamera()
      }

      dt = Mecanum(map, "fl", "fr", "bl", "br")
      addComponents(dt, camera)
    }
  }

  override fun onInit(scope: CoroutineScope){
    
    val dash = FtcDashboard.getInstance()
    telemetry = MultipleTelemetry(telemetry, dash.getTelemetry())
    dash.startCameraStream(camera.webcam, 30.0)
    
  }

  override fun onStart(scope: CoroutineScope) = runBlocking {

    while(!camera.cameraRunning) yield()

  }

  override fun onUpdate(scope: CoroutineScope){
    camera.kp = kCam
    val center = camera.alignmentPipeline.center
    if (center != null){
      dt.drive(center.x * kx, (followDist - camera.dist) * ky, center.x * kw)
    } else{
      dt.drive(0.0, 0.0, 0.0)
    }
  }
}

