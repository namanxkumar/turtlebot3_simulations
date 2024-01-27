#!/usr/bin/env python3
import rospy
import rospkg

from typing import List, Tuple
import random

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState, SpawnModel
from people_msgs.msg import People, Person
from geometry_msgs.msg import Pose, Point, Quaternion, Twist


class Box:
    def __init__(
        self,
        name: str,
        initial_position_x: float,
        initial_position_y: float,
        get_model_state_service: rospy.ServiceProxy,
        set_model_state_service: rospy.ServiceProxy,
        model_spawn_service: rospy.ServiceProxy,
        model_xml: str = None,
    ) -> None:
        self.name = name

        self.pose_update_duration = rospy.Duration(5)

        if model_xml is None:
            rospack = rospkg.RosPack()
            package_path = rospack.get_path("turtlebot3_gazebo")
            with open(f"{package_path}/urdf/box.sdf", "r") as file:
                self.model_xml = file.read()
        else:
            self.model_xml = model_xml

        self.get_model_state_service = get_model_state_service
        self.set_model_state_service = set_model_state_service
        self.model_spawn_service = model_spawn_service

        self.spawned = self.spawn_model(
            initial_position_x=initial_position_x, initial_position_y=initial_position_y
        )

        # self._set_pose(
        #     position_x=initial_position_x,
        #     position_y=initial_position_y,
        #     linear_velocity_x=0,
        #     linear_velocity_y=0,
        # )

        # self.list_positions_x = [0.5, 0.5, -0.5, -0.5]
        # self.list_positions_y = [0.5, -0.5, -0.5, 0.5]
        # self.list_linear_velocities_x = [0, -0.5, 0, 0.5]
        # self.list_linear_velocities_y = [-0.5, 0, 0.5, 0]

        # assert (
        #     len(self.list_positions_x)
        #     == len(self.list_positions_y)
        #     == len(self.list_linear_velocities_x)
        #     == len(self.list_linear_velocities_y)
        # )

        # self.pose_index = 0

    @property
    def current_state(self) -> Tuple[Pose, Twist]:
        rospy.wait_for_service("/gazebo/get_model_state")
        response = self.get_model_state_service(self.name, "world")
        if response.success:
            return response.pose, response.twist
        else:
            raise Exception(f"Failed to get state of model {self.name}")

    def _get_random_velocity(self) -> float:
        return random.uniform(-0.5, 0.5)

    def spawn_model(self, initial_position_x: float, initial_position_y: float) -> bool:
        pose = Pose(
            position=Point(
                x=initial_position_x,
                y=initial_position_y,
                z=0.1,
            ),
            orientation=Quaternion(
                x=0,
                y=0,
                z=0,
                w=0,
            ),
        )
        response = self.model_spawn_service(
            model_name=self.name,
            model_xml=self.model_xml,
            robot_namespace="",
            initial_pose=pose,
        )
        if not response.success:
            if "entity already exists" in response.status_message:
                print(f"Model {self.name} already spawned")
                return True
            print(f"Failed to spawn model {self.name}")
            return False
        print(f"Spawned model {self.name}")
        return True

    def initialize_pose_update_timer(self):
        rospy.Timer(self.pose_update_duration, self.move)

    # @property
    # def list_index(self):
    #     return self.pose_index % len(self.list_positions_x)

    def move(self, _=None):
        status = self._set_velocities(
            linear_velocity_x=self._get_random_velocity(),
            linear_velocity_y=self._get_random_velocity(),
        )
        if status:
            print(f"Moved model {self.name}")
        else:
            print(f"Failed to move model {self.name}")
        # status = self._set_pose(
        #     position_x=self.list_positions_x[self.list_index],
        #     position_y=self.list_positions_y[self.list_index],
        #     linear_velocity_x=self.list_linear_velocities_x[self.list_index],
        #     linear_velocity_y=self.list_linear_velocities_y[self.list_index],
        # )
        # if status:
        #     self.pose_index += 1

    def _set_velocities(self, linear_velocity_x: float, linear_velocity_y: float):
        pose, _ = self.current_state
        success = self._set_pose(
            position_x=pose.position.x,
            position_y=pose.position.y,
            linear_velocity_x=linear_velocity_x,
            linear_velocity_y=linear_velocity_y,
        )
        if not success:
            print(f"Failed to set model {self.name} velocities")
            return False
        print(f"Set model {self.name} velocities to {linear_velocity_x}, {linear_velocity_y}")
        return True

    def _set_pose(
        self,
        position_x: float,
        position_y: float,
        linear_velocity_x: float,
        linear_velocity_y: float,
    ):
        if not self.spawned:
            print(f"Model {self.name} not spawned")
            return False

        state_msg = ModelState()
        state_msg.model_name = self.name
        state_msg.pose.position.x = position_x
        state_msg.pose.position.y = position_y
        state_msg.pose.position.z = 0.1
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 0
        state_msg.twist.linear.x = linear_velocity_x
        state_msg.twist.linear.y = linear_velocity_y
        state_msg.twist.linear.z = 0
        state_msg.twist.angular.x = 0
        state_msg.twist.angular.y = 0
        state_msg.twist.angular.z = 0

        response = self.set_model_state_service(state_msg)
        if not response.success:
            print(f"Failed to set model {self.name} pose")
            print(response.status_message)
            return False
        print(f"Set model {self.name} pose")
        return True

    @property
    def person_msg(self):
        pose, twist = self.current_state
        person_msg = Person()
        person_msg.name = self.name
        person_msg.position.x = pose.position.x
        person_msg.position.y = pose.position.y
        person_msg.position.z = pose.position.z
        person_msg.velocity.x = twist.linear.x
        person_msg.velocity.y = twist.linear.y
        person_msg.velocity.z = twist.linear.z
        person_msg.reliability = 1
        person_msg.tagnames = [self.name]
        person_msg.tags = [self.name]
        return person_msg


class Boxes:
    def __init__(self, boxes: List[Box]) -> None:
        self.boxes = boxes

        self.publish_people_duration = rospy.Duration(0.1)

        self.people_publisher = rospy.Publisher("/people", People, queue_size=1)

    def initialize_people_publish_timer(self):
        rospy.Timer(self.publish_people_duration, self.publish_people)

    def initialize_boxes_pose_update_timers(self):
        for box in self.boxes:
            box.initialize_pose_update_timer()

    def publish_people(self, _=None):
        people_msg = People()
        people_msg.header.stamp = rospy.Time.now()
        people_msg.header.frame_id = "map"
        people_msg.header.seq = 0
        people_msg.people = [box.person_msg for box in self.boxes]

        self.people_publisher.publish(people_msg)


def main():
    rospy.init_node("set_pose")

    rospy.wait_for_service("/gazebo/set_model_state")
    rospy.wait_for_service("/gazebo/get_model_state")
    rospy.wait_for_service("/gazebo/spawn_urdf_model")

    get_model_state_service = rospy.ServiceProxy(
        "/gazebo/get_model_state", GetModelState
    )
    set_model_state_service = rospy.ServiceProxy(
        "/gazebo/set_model_state", SetModelState
    )
    model_spawn_service = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    boxes = Boxes(
        boxes=[
            Box(
                "box1",
                initial_position_x=-4,
                initial_position_y=4,
                get_model_state_service=get_model_state_service,
                set_model_state_service=set_model_state_service,
                model_spawn_service=model_spawn_service,
            ),
            Box(
                "box2",
                initial_position_x=-1,
                initial_position_y=4,
                get_model_state_service=get_model_state_service,
                set_model_state_service=set_model_state_service,
                model_spawn_service=model_spawn_service,
            ),
            Box(
                "box3",
                initial_position_x=6,
                initial_position_y=1,
                get_model_state_service=get_model_state_service,
                set_model_state_service=set_model_state_service,
                model_spawn_service=model_spawn_service,
            ),
            Box(
                "box4",
                initial_position_x=-6.5,
                initial_position_y=-1,
                get_model_state_service=get_model_state_service,
                set_model_state_service=set_model_state_service,
                model_spawn_service=model_spawn_service,
            ),
        ]
    )

    boxes.initialize_boxes_pose_update_timers()
    boxes.initialize_people_publish_timer()

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
