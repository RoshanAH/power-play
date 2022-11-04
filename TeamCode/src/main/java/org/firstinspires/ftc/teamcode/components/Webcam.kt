package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap
import org.openftc.easyopencv.OpenCvWebcam
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraRotation
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.teamcode.core.Component
import org.firstinspires.ftc.teamcode.pipelines.SignalDetector
import org.firstinspires.ftc.teamcode.pipelines.AlignmentDetector
import kotlinx.coroutines.CoroutineScope
import java.util.concurrent.TimeUnit

class Webcam(map: HardwareMap, webcam: String, mount: String) : Component {

  var exposure = 0L
  var height = 0.0

  val webcam = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName::class.java, webcam))
  val mount = map.servo.get(mount)

  val signalPipeline = SignalDetector()
  val alignmentPipeline = AlignmentDetector()

  var theta = 0.0

  var cameraRunning = false
    private set

  fun startCamera(){
    webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
        override fun onOpened(){
          webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
          // webcam.exposureControl.mode = ExposureControl.Mode.Manual 
          // webcam.exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS)
          Thread.sleep(1000)
          cameraRunning = true
        }

        override fun onError(errorCode: kotlin.Int) {}
    })
  }

  override fun init(scope: CoroutineScope) {}
  override fun start(scope: CoroutineScope) {}


  override fun update(scope: CoroutineScope) {
    mount.setPosition(0.5 + (90.0 - theta) / (300.0))
  }
}
