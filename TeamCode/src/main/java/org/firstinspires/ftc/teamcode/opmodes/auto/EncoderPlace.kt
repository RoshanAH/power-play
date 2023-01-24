package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.roshanah.jerky.math.deg
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.robots.Ptoneigh
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.launch
import kotlinx.coroutines.delay
import kotlinx.coroutines.yield
import kotlin.math.abs

@Autonomous(preselectTeleOp="MainOp")
class EncoderPlace : BaseOpmode(){
  val robot = Ptoneigh({ opModeIsActive() })
  var signal = 2 // leave this hardcoded for now since we dont have vision programmed in yet
  override fun setRobot() = robot

  override fun onInit(scope: CoroutineScope){
    robot.claw.claw.setPosition(robot.claw.close)
    robot.claw.close()
  }

  override fun onStart(scope: CoroutineScope){
    robot.claw.close()

    scope.launch{
      robot.turret.targetHeight = 0.2
      launch { // this is being run asynchronously hence the launch call
        delay(1000L)
        robot.turret.targetHeight = robot.high 
        while(robot.turret.height < robot.turret.lowestRotationHeight) yield() // wait for slides to be up before moving turret
        robot.turret.targetTheta = 135.0 // degrees
        while(abs(robot.turret.targetTheta - robot.turret.theta) > 10.0) yield() // wait for turret to move before extending
        robot.claw.targetExtension = 7.5
      }
      robot.forward(56.0)
      robot.turnTo(-90.0.deg)
      place()

      for(i in 0 until 5){ // lower this if we start running out of time
        val height = (4 - i) * robot.coneHeight
        launch{
          robot.claw.targetExtension = 0.0
          sleep(250L) // wait for claw to retract
          robot.turret.apply{
            targetTheta = 0.0
            targetHeight = height
            while (abs(targetHeight - height) < 2.0) yield() // wait for slides to move down before reextending
          }
          robot.claw.targetExtension = robot.claw.maxExtension
        }
        // move forward a bit
        // close claw
        // raise slides up a bit

        // launch { // this is being run asynchronously hence the launch call
        //   delay(500L)
        //   robot.turret.targetHeight = robot.high 
        //   while(robot.turret.height < robot.turret.lowestRotationHeight) yield() // wait for slides to be up before moving turret
        //   robot.turret.targetTheta = 135.0 // degrees
        //   while(abs(robot.turret.targetTheta - robot.turret.theta) > 10.0) yield() // wait for turret to move before extending
        //   robot.claw.targetExtension = 7.5
        // }

        // move back the same amount as moved forward
      }

      // park
    }
    
  }

  suspend fun place(){
    robot.turret.targetHeight -= 0.15
    robot.claw.open()
  }


}
