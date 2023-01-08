package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.roshanah.jerky.utils.DriveValues
import com.roshanah.jerky.math.deg
import com.roshanah.jerky.math.Vec2
import org.firstinspires.ftc.teamcode.components.Webcam
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.robots.Jaws
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.launch
import com.acmerobotics.dashboard.FtcDashboard
import kotlinx.coroutines.yield
import kotlin.math.sqrt

@Autonomous
class PlaceX : BaseOpmode(){
  val robot = Jaws().apply { active = { opModeIsActive() } }

  override fun setRobot() = robot

  override fun onInit(scope: CoroutineScope){
    robot.slides.claw.setPosition(robot.slides.close)
    robot.slides.close()

    FtcDashboard.getInstance().startCameraStream(robot.camera.webcam, 30.0)

    robot.camera.startCamera()
  }

  override fun onStart(scope: CoroutineScope){
    scope.launch{
      while(opModeIsActive() && !robot.camera.cameraRunning){
        telemetry.addLine("Waiting for camera to start...")
        telemetry.update()
        yield()
      }
      robot.camera.webcam.setPipeline(robot.camera.alignmentPipeline)
      robot.camera.alignment = Webcam.Alignment.POLES
      robot.slides.targetPosition = robot.high

      val approach = robot.follow{
        to(-37.0, 5.0, 0.0)
        val angle = 10.0.deg
        turnTo(0.0, 30.0, angle)
      }

      robot.place()

      robot.follow(vi=approach.vf){
        stop()
      }

      robot.drivetrain.move(DriveValues.zero)
    }
  }
}
