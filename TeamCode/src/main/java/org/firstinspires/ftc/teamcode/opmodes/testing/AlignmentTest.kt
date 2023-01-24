package org.firstinspires.ftc.teamcode.opmodes.testing

import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.components.Mecanum
import org.firstinspires.ftc.teamcode.components.Webcam
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
import com.roshanah.jerky.math.deg
import kotlinx.coroutines.CoroutineScope
import kotlin.math.abs
import kotlin.math.sign

@TeleOp(group = "testing")
@Config
class AlignmentTest : BaseOpmode(){

  companion object{
    @JvmField var alignment = Webcam.Alignment.NONE
    @JvmField var rangeOfMotion = 270.0
  }

  lateinit var camera: Webcam

  override fun setRobot() = object: Robot(){
    override fun mapHardware(map: HardwareMap){
      val turret = map.dcMotor.get("turret")
      camera = Webcam(map, 
        "Webcam 1", 
        "webcamEncoder", 
        "mount", 
        { 0.0 },
        { (turret.getCurrentPosition() / 4.2666).deg }
      )
      addComponents(camera)
    }
  }

  override fun onInit(scope: CoroutineScope){
    camera.startCamera()
    FtcDashboard.getInstance().startCameraStream(camera.webcam, 30.0)
    camera.webcam.setPipeline(camera.alignmentPipeline)
    camera.alignment = Webcam.Alignment.POLES
  } 

  override fun onUpdate(scope: CoroutineScope){
    camera.alignment = alignment
    val projection = camera.target
    // pose.apply { println("x: ${pos.x}, y: ${pos.y}, z: ${pos.z}, theta: ${dir.theta.deg}, phi: ${dir.phi.deg}") }
    telemetry.addData("projection", projection)
    telemetry.addData("camera position", camera.pos)
    telemetry.addData("distance", projection?.magnitude)
    telemetry.update()
  }
}
