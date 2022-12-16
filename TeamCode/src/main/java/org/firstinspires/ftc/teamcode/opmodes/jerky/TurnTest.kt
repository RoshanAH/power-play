package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.roshanah.jerky.profiling.buildProfile
import com.roshanah.jerky.math.deg
import com.roshanah.jerky.math.Pose
import com.roshanah.jerky.utils.DriveValues
import com.acmerobotics.dashboard.FtcDashboard
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.robots.Jaws
import org.firstinspires.ftc.teamcode.components.Webcam
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.launch
import kotlinx.coroutines.yield
import kotlinx.coroutines.delay

@Autonomous
@Disabled
class TurnTest : BaseOpmode(){
  val robot = Jaws()
  override fun setRobot() = robot

  override fun onInit(scope: CoroutineScope){
    robot.slides.claw.setPosition(robot.slides.close)
    robot.slides.close()
  }

  override fun onStart(scope: CoroutineScope){
    scope.launch{

      robot.follow{
        to(0.0, 30.0, 2.0)
        stop()
      } 

      delay(250L)

      robot.follow(robot.imu.heading){
        to(Pose(-((theta + 90.0.deg).dir) * 20.0, 0.0))
        displace(5.0)
        // turnTo(0.0, 30.0, -90.0.deg - theta)
        // to(40.0, 0.0, 0.0)
        stop()
      }

      robot.drivetrain.move(DriveValues.zero)

    }

  }

  override fun onUpdate(scope: CoroutineScope){
    telemetry.addData("heading", robot.imu.heading)
    telemetry.update()
  }
}
