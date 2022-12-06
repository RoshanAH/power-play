package org.firstinspires.ftc.teamcode.components

import com.roshanah.jerky.utils.DriveConstants
import com.roshanah.jerky.utils.DriveValues
import com.roshanah.jerky.math.Pose
import com.roshanah.jerky.math.rotation
import com.roshanah.jerky.math.Vec2
import com.roshanah.jerky.profiling.Derivatives
import com.qualcomm.robotcore.hardware.Gamepad
import kotlin.math.abs
import kotlin.math.sqrt
import kotlin.math.sign

class PSVAController(var constants: DriveConstants) {
  var lastTime = System.nanoTime() * 1e-9
  var vel = Pose.zero
    private set
  var wheels = Derivatives(DriveValues.zero, DriveValues.zero, DriveValues.zero)
    private set

  fun update(tVel: Pose){
    val time = System.nanoTime() * 1e-9
    val deltaTime = time - lastTime
    lastTime = time
    
    if (tVel == Pose.zero){
      vel = Pose.zero
      wheels = Derivatives(DriveValues.zero, DriveValues.zero, DriveValues.zero)
      return
    }
  

    val difference = tVel - vel
    val r = constants.trackRadius
    val w = vel.heading

    val alpha: Double
    val a: Double

    if (difference.heading != 0.0) {
      val relation = (difference.pos).magnitude / abs(difference.heading)
      alpha = constants.maxAcceleration / (relation * sqrt(2.0) + 2 * r)
      a = relation * alpha

    } else if (difference.pos.magnitude != 0.0){
      alpha = 0.0
      a = constants.maxAcceleration / sqrt(2.0)
    } else{
      alpha = 0.0 
      a = 0.0
    }

    val accel = Pose(if (a == 0.0) Vec2.zero else difference.pos.unit * a, alpha * sign(difference.heading))
    val dv = accel * deltaTime

    if (dv.pos.magnitude > difference.pos.magnitude)
      vel = tVel
    else
      vel += dv

    wheels =
        Derivatives(
            DriveValues(
                0.0,
                0.0,
                0.0,
                0.0
            ), // this integral is impossible to evaluate manually and really not useful so we'll
            // just put zeros here
            DriveValues( // inverse kinematics for velocity
                vel.y + vel.x - (2 * r * w),
                vel.y - vel.x + (2 * r * w),
                vel.y - vel.x - (2 * r * w),
                vel.y + vel.x + (2 * r * w),
            ),
            DriveValues( // derivative of our velocity
                accel.y + accel.x - (2 * r * alpha),
                accel.y - accel.x + (2 * r * alpha),
                accel.y - accel.x - (2 * r * alpha),
                accel.y + accel.x + (2 * r * alpha),
            ),
        )
  }
  fun update(gamepad: Gamepad){
    update(pose(gamepad))
  }

  fun pose(gamepad: Gamepad) = Pose(
    gamepad.left_stick_x.toDouble(), 
   -gamepad.left_stick_y.toDouble(), 
   -gamepad.right_stick_x.toDouble()
  ) * constants.maxVelocity / sqrt(2.0)
  
}
