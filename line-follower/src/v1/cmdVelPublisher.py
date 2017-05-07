from geometry_msgs.msg import Twist, Vector3
import rospy

class CmdVelPublisher(object):

    def __init__(self):
        super(CmdVelPublisher, self).__init__()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.linearVector = Vector3(x=0.05, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.state = {0:self.forward,
                      1:self.leftTurn,
                      2:self.rightTurn,
                      3:self.stopState}
        print "Initialized CmdVelPublisher"

        rospy.on_shutdown(self.stop_robot)

    def stop_robot(self):
        self.state[3].__call__()
        self.sendMessage()

    def forward(self):
        """ Sets the velocity to forward """
        print('forward')
        self.linearVector  = Vector3(x=0.3, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=0.0)

    def leftTurn(self):
        print('leftTurn')
        """ Sets the velocity to turn left """
        self.linearVector  = Vector3(x=0.3, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=1.5)

    def rightTurn(self):
        print('rightTurn')
        """ Sets the velocity to turn right """
        self.linearVector  = Vector3(x=0.3, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=-1.5)

    def stopState(self):
        """ Sets the velocity to stop """
        # print('stop')
        self.linearVector  = Vector3(x=0.0, y=0.0, z=0.0)
        self.angularVector = Vector3(x=0.0, y=0.0, z=0.0)

    def sendMessage(self):
        """ Publishes the Twist containing the linear and angular vector """
        self.pub.publish(Twist(linear=self.linearVector, angular=self.angularVector))
