import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import com.autom8ed.lr2.MainActivity
import com.segway.robot.sdk.base.action.RobotAction

class IntentReceiver : BroadcastReceiver() {
    override fun onReceive(context: Context, intent: Intent) {
        if (intent.action == RobotAction.TransformEvent.ROBOT_MODE) {
            val activityIntent = Intent(
                context,
                MainActivity::class.java
            )
            context.startActivity(activityIntent)
        }
    }
}