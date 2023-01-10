package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.roshanah.jerky.utils.DriveValues
import com.roshanah.jerky.math.deg
import com.roshanah.jerky.math.Vec2
import com.roshanah.jerky.math.Pose
import org.firstinspires.ftc.teamcode.components.Webcam
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.robots.Jaws
import org.firstinspires.ftc.teamcode.pipelines.AlignmentDetector
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.launch
import com.acmerobotics.dashboard.FtcDashboard
import kotlinx.coroutines.yield
import kotlinx.coroutines.delay
import kotlin.math.sqrt

@Autonomous
class PlaceX : BaseOpmode(){
  val robot = Jaws().apply { active = { opModeIsActive() } }

  override fun setRobot() = robot

  override fun onInit(scope: CoroutineScope){
    robot.slides.claw.setPosition(robot.slides.close)
    robot.camera.mount.setPosition(0.5)
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

      robot.follow{
        to(-40.0, 5.0, 0.0)
        val angle = 25.0.deg
        turnTo(0.0, 30.0, angle)
      }

      robot.place()
      delay(250L)

      launch {
        delay(1000L)
        robot.slides.targetPosition = 0.01
        robot.camera.alignment = Webcam.Alignment.BLUE_CONES
      }

      robot.follow(theta=robot.imu.heading){
        to(Pose(-(90.0.deg + theta).dir * 30.0, 0.0))
        turnTo(10.0, 5.0, -theta)
        to(0.0, 30.0, 0.0)
        turnTo(20.0, 0.0, -90.0.deg)
      }

      robot.coneStack(4, -90.0.deg)


      robot.drivetrain.move(DriveValues.zero)
    }
  }
}
