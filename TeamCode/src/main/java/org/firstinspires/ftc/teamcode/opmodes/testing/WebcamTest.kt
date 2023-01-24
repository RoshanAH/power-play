package org.firstinspires.ftc.teamcode.opmodes.testing

import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.components.Mecanum
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraRotation
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.FtcDashboard
import kotlinx.coroutines.CoroutineScope
import kotlin.math.abs
import kotlin.math.sign

@TeleOp(group = "testing")
@Config
class WebcamTest : LinearOpMode(){

  companion object {
    @JvmField var ticksPerDegree = 1.0
  }

  override fun runOpMode(){
    val map = hardwareMap
    val webcam = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName::class.java, "Webcam 1"))
    val mount = map.servo.get("mount")
    val encoder = map.dcMotor.get("webcamEncoder")
    var cameraStarted = false

    webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
        override fun onOpened(){
          webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
          Thread.sleep(1000)
          cameraStarted = true
        }
        override fun onError(errorCode: kotlin.Int) {}
    })

    while(!cameraStarted) {
      telemetry.addLine("waiting for camera to start")
      telemetry.update()
    }

    FtcDashboard.getInstance().startCameraStream(webcam, 30.0)

    waitForStart()    

    mount.setPosition(0.5)
    sleep(500)
    encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
    encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

    mount.controller.pwmDisable()
    mount.controller.close()

    while(opModeIsActive()){
      val raw = encoder.getCurrentPosition()
      val phi = raw / ticksPerDegree

      telemetry.addData("raw", raw)
      telemetry.addData("phi", phi)
      telemetry.update()
    }
  }
}
