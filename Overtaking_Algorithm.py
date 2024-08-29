
import rospy
from autoware_msgs.msg import DetectedObjectArray
from autoware_msgs.msg import DetectedObject
import numpy as np
from hmcl_msgs.msg import Lane
import time
from hmcl_msgs.msg import Waypoint

import math
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
import math
from tf.transformations import quaternion_from_euler


class Model_Predictive():
    def __init__(self):
        rospy.init_node("Overtaking")
        self.target_sub=rospy.Subscriber('/tracking_side/objects', DetectedObjectArray, self.detected_object_callback)
        self.global_traj_sub=rospy.Subscriber('/local_traj',Lane,self.global_traj_callback)
        # self.global_traj_sub=rospy.Subscriber('/optimal_traj',LaneArray,self.global_traj_callback1)
        self.overtaking_traj_pub=rospy.Publisher('/local_traj1',Lane, queue_size=1)
        self.marker_pub=rospy.Publisher('/over_traj_viz',MarkerArray, queue_size=1)

        self.model_predicted_num = 5
        self.dt = 0.1
        self.target_x = 0
        self.target_y = 0
        self.target_velocity = 0
        self.target_angular_z = 0
        self.control = np.zeros(2)
        self.state=[0]*4
        rate = rospy.Rate(1)
        self.model_prediction_x = []
        self.model_prediction_y = []
        self.repulsed_potential_field_point_x=[]
        self.repulsed_potential_field_point_y=[]
        self.vx_data = []
        self.vy_data = []
        self.target_x_data=[]
        self.target_y_data=[]
        self.target_vel_x={}
        self.target_vel_y={}
        self.plot_done={}
        self.plot_done = False
        self.rate = rospy.Rate(1)
     
        rospy.Timer(rospy.Duration(1), self.main_loop)
      
        self.bubble_param=1.3
        self.object_dic={}
        self.flag_run_potential_field = False  # Initialize the flag as False

        rospy.Timer(rospy.Duration(0.3), self.timer_callback)

        rospy.spin()
    def main_loop(self, event):
        # Put the main loop logic here
        pass

    def detected_object_callback(self, data):
        # rospy.loginfo("DetectedObjectArray received")
        self.target_veh_dic_x = {}
        self.target_veh_dic_y = {}
        self.obj = []
        self.objects_data = data

        for obj in data.objects:
            self.target_velocity_x=obj.velocity.linear.x
            self.target_velocity_y=obj.velocity.linear.y
            self.target_x = obj.pose.position.x
            self.target_y = obj.pose.position.y
            self.target_orientation_x=obj.pose.orientation.x 
            self.target_orientation_y=obj.pose.orientation.y 
            self.target_orientation_z=obj.pose.orientation.z 
            self.target_orientation_w=obj.pose.orientation.w
            self.target_yaw = obj.velocity.angular.z

            # obj 대신 obj의 고유 식별자(예: obj.id 또는 obj.label)를 키로 사용
            obj_key = obj.id
            self.detected_object_kinematic(obj.label)  # 또는 obj.label, obj.name 등 고유 식별자

            # if obj_key not in self.target_vel_x:
            #     self.target_vel_x[obj_key] = []
            # self.target_vel_x[obj_key].append(self.target_velocity_x)

            # if obj_key not in self.target_vel_y:
            #     self.target_vel_y[obj_key] = []
            # self.target_vel_y[obj_key].append(self.target_velocity_y)

            # if len(self.target_vel_x[obj_key]) > 1000:
            #     if not self.plot_done[obj_key]:
            #         self.make_lstm(obj_key)
            #     self.run_lstm(obj_key)
            #     self.plot_done[obj_key] = True  # 특정 obj_key에 대해 완료되었음을 표시
        self.flag_run_potential_field = True
        # if len(data.objects) == 0:
        #     self.flag_run_potential_field = False

        # self.Intuitive_Artifical_potential_field_2()
  
    def timer_callback(self, event):
        # Only call the function if the flag is set to True
        if self.flag_run_potential_field:
            self.Intuitive_Artifical_potential_field_2()
                
    # def global_traj_callback1(self,data):
    #     print('global')
    #     self.cx=[]
    #     self.cy=[]
    #     self.cqx=[]
    #     self.cqy=[]
    #     self.cqz=[]
    #     self.cqw=[]
    #     for i in data.lanes:
    #         for j in i.waypoints:
    #             self.cx.append(j.pose.pose.position.x)
    #             self.cy.append(j.pose.pose.position.y)
    #             self.cqx.append(j.pose.pose.orientation.x)
    #             self.cqy.append(j.pose.pose.orientation.y)
    #             self.cqz.append(j.pose.pose.orientation.z)
    #             self.cqw.append(j.pose.pose.orientation.w)

    

   
    def global_traj_callback(self,data):
        self.cx=[]
        self.cy=[]
        self.cqx=[]
        self.cqy=[]
        self.cqz=[]
        self.cqw=[]
        
        for j in data.waypoints:
            self.cx.append(j.pose.pose.position.x)
            self.cy.append(j.pose.pose.position.y)
            self.cqx.append(j.pose.pose.orientation.x)
            self.cqy.append(j.pose.pose.orientation.y)
            self.cqz.append(j.pose.pose.orientation.z)
            self.cqw.append(j.pose.pose.orientation.w)

    
        

            #set mode simulation or Detect(ROS bag)
        # self.model_prediction_x = []
        # self.model_prediction_y = []
            
    
        
  


            
    def detected_object_kinematic(self,obj):
        model_prediction_x = self.target_x
        target_velocity_x = self.target_velocity_x
        target_yaw = self.target_yaw
        model_prediction_y = self.target_y
        target_velocity_y = self.target_velocity_y
        target_angular_z = self.target_angular_z
        self.model_prediction_x = []
        self.model_prediction_y = []
        for i in range(self.model_predicted_num):
            self.model_prediction_x.append(model_prediction_x)
            model_prediction_x += target_velocity_x
            target_yaw += target_angular_z
            target_velocity_x = self.target_velocity * math.cos(target_yaw)
        self.target_veh_dic_x[obj]=self.model_prediction_x
        for j in range(self.model_predicted_num):
            self.model_prediction_y.append(model_prediction_y)
            model_prediction_y += target_velocity_y
            target_yaw += target_angular_z
            target_velocity_y = self.target_velocity * math.sin(target_yaw)
        self.target_veh_dic_y[obj]=self.model_prediction_y

    



    def yaw_to_quaternion(self,yaw):
        # yaw, pitch, roll 값에서 yaw만 사용해 변환
        roll = 0.0
        pitch = 0.0
        
        # 쿼터니언으로 변환
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        return quaternion

   



 
    def cartesian_to_frenet(self, centerline, point):
    # Convert centerline and point to numpy arrays
        centerline = np.array(centerline)
        diffs = np.diff(centerline, axis=0)
        dists = np.linalg.norm(diffs, axis=1)
        arclength = np.insert(np.cumsum(dists), 0, 0.0)

        # Initialize variables
        point = np.array(point)
        min_dist = float('inf')
        s, l = 0, 0

        for i in range(len(diffs)):
            p1 = centerline[i]
            p2 = centerline[i + 1]

            # Vector along the centerline segment
            line_vec = p2 - p1
            point_vec = point - p1
            line_len = np.linalg.norm(line_vec)
            proj_length = np.dot(point_vec, line_vec) / line_len
            proj_point = p1 + (proj_length / line_len) * line_vec
            
            # Calculate distance from the point to the projection on the centerline
            dist = np.linalg.norm(point - proj_point)
            
            # Determine the sign of l using the cross product
            cross_prod = np.cross(line_vec, point - proj_point)
            side_sign = np.sign(cross_prod)
            
            if dist < min_dist:
                min_dist = dist
                s = arclength[i] + proj_length
                l = side_sign * dist
        
        return s, l
      
    def frenet_to_cartesian(self, centerline, s, l):
    # 중심선의 아크 길이 계산
        centerline = np.array(centerline)
        diffs = np.diff(centerline, axis=0)
        dists = np.linalg.norm(diffs, axis=1)
        arclength = np.insert(np.cumsum(dists), 0, 0.0)

        # s에 해당하는 세그먼트 인덱스 찾기
        segment_index = np.searchsorted(arclength, s) - 1
        if segment_index < 0:
            segment_index = 0
        elif segment_index >= len(centerline) - 1:
            segment_index = len(centerline) - 2

        p1 = centerline[segment_index]
        p2 = centerline[segment_index + 1]

        # 세그먼트의 방향 벡터 및 단위 벡터 계산
        segment_vector = p2 - p1
        segment_length = dists[segment_index]
        segment_unit_vector = segment_vector / segment_length

        # s에 대한 기본점 계산
        base_point = p1 + segment_unit_vector * (s - arclength[segment_index])

        # 법선 벡터 계산 (세그먼트에 수직)
        normal_vector = np.array([-segment_unit_vector[1], segment_unit_vector[0]])

        # 카르테시안 좌표 계산
        cartesian_point = base_point + normal_vector * l

        return cartesian_point[0], cartesian_point[1]


 
    def Intuitive_Artifical_potential_field_2(self):
        # self.local_cx=[]
        # self.local_cy=[]
        # for i in self.cx:    
        #     self.local_cx.append(i)
        # for i in self.cy:
        #     self.local_cy.append(i)# Oval
        # self.radius=10
        start_time=time.time()
        self.gain=10
        self.target_velocity = 300
        self.sigma=1/self.target_velocity
        self.radius=round(np.sqrt(1/self.sigma)+1,3)*10000000000
        potential_field_point={}
        self.range_point_x={}
        self.range_point_y={}
        self.repulsed_potential_field_point_x={}
        self.repulsed_potential_field_point_y={}
        print(len(self.objects_data.objects))
        for obj in self.objects_data.objects:
    # 고유 식별자를 사용
            obj_key = obj.label # 혹은 obj.label, obj.name 등 고유 식별자

            potential_field_point[obj_key] = []
            self.range_point_x[obj_key] = []
            self.range_point_y[obj_key] = []
            self.repulsed_potential_field_point_x[obj_key] = []
            self.repulsed_potential_field_point_y[obj_key] = []

            for i in range(len(self.cx)):
                distance = ((self.target_veh_dic_x[obj_key][2] - self.cx[i])**2 + (self.target_veh_dic_y[obj_key][2] - self.cy[i])**2)**0.5
                if distance < self.radius:
                    potential_field_point[obj_key].append([self.cx[i], self.cy[i]])
                # if self.radius < distance < self.radius * 2:
                #     self.range_point_x[obj_key].append(self.cx[i])
                #     self.range_point_y[obj_key].append(self.cy[i])
            print(len(potential_field_point[obj_key]))
            point = [potential_field_point[obj_key][len(potential_field_point[obj_key])// 2]]
            print(point)

            center_s, center_d = self.cartesian_to_frenet(potential_field_point[obj_key], point)
            print(center_s)
            s1, d1 = self.cartesian_to_frenet(potential_field_point[obj_key], [self.target_veh_dic_x[obj_key][0], self.target_veh_dic_y[obj_key][0]])
            print(d1)
            for j in potential_field_point[obj_key]:
                s, d = self.cartesian_to_frenet(potential_field_point[obj_key], j)
                for i in range(self.model_predicted_num):
                    if d1 >= 0:
                        d -= self.gain * np.sqrt(np.maximum(0, (1 - self.sigma * (s - s1)**2)))
                    else:
                        d += self.gain * np.sqrt(np.maximum(0, (1 - self.sigma * (s - s1)**2)))

                self.repulsed_s = s
                self.repulsed_d = d
                # print(d)
                self.repulsed_x, self.repulsed_y = self.frenet_to_cartesian(potential_field_point[obj_key], self.repulsed_s, self.repulsed_d)
                print([self.repulsed_x,self.repulsed_y])
                self.repulsed_potential_field_point_x[obj_key].append(self.repulsed_x)
                self.repulsed_potential_field_point_y[obj_key].append(self.repulsed_y)
            
            self.Bezier_Curve(obj_key)

        end_time=time.time()
        print(f'execution_time{start_time-end_time}')
        if len(self.objects_data.objects) == 0:
            self.OG_PUB()

     

    def setColor(self, color, r, g, b, a):
        color.r = r
        color.g = g
        color.b = b
        color.a = a

    def visualize_local_path(self, waypoints):
        traj_marker_color = ColorRGBA()
        self.setColor(traj_marker_color, 1.0, 0.0, 0.0, 0.5)
        in_bank_color = ColorRGBA()
        self.setColor(in_bank_color, 1.0, 1.0, 0.0, 0.5)
        
        marker_array = MarkerArray()
        
        for i, waypoint in enumerate(waypoints):
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = "map"
            marker.id = 50000 + i
            marker.ns = "ltraj"
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = waypoint.pose.pose.position.x
            marker.pose.position.y = waypoint.pose.pose.position.y
            marker.pose.position.z = waypoint.pose.pose.position.z
            marker.pose.orientation.x = waypoint.pose.pose.orientation.x
            marker.pose.orientation.y = waypoint.pose.pose.orientation.y
            marker.pose.orientation.z = waypoint.pose.pose.orientation.z
            marker.pose.orientation.w = waypoint.pose.pose.orientation.w
            
            if hasattr(waypoint, 'bank') and waypoint.bank:
                marker.color = in_bank_color
            else:
                marker.color = traj_marker_color
            
            marker.lifetime = rospy.Duration(0.1)
            marker.scale.x = 0.7
            marker.scale.y = 0.4
            marker.scale.z = 0.1
            
            marker_array.markers.append(marker)
        
        return marker_array

        

    def OG_PUB(self):
        trajectory = Lane()
        i = 0
        # for x, y, qx, qy. qz. qw in zip(self.cx, self.cy, self.cqx, self.cqy, self.cqz, self.cqw):
        while i < len(self.cx):
            waypoint = Waypoint()
            waypoint.pose.pose.position.x = self.cx[i]
            waypoint.pose.pose.position.y = self.cy[i]
            waypoint.pose.pose.orientation.x = self.cqx[i]
            waypoint.pose.pose.orientation.y = self.cqy[i]
            waypoint.pose.pose.orientation.z = self.cqz[i]
            waypoint.pose.pose.orientation.w = self.cqw[i]
            trajectory.waypoints.append(waypoint)
            i += 1
        self.overtaking_traj_pub.publish(trajectory)
        
        # Visualize the local path using markers
        marker_array = self.visualize_local_path(trajectory.waypoints)
        self.marker_pub.publish(marker_array)
        print("work2222")



    def Bezier_Curve(self, obj):
        print("work!")
        # Bezier 처리
        repulsed_x = np.array(self.repulsed_potential_field_point_x[obj])
        repulsed_y = np.array(self.repulsed_potential_field_point_y[obj])
        
        num = len(repulsed_x)
        
        # 점 계산 (각각 1/3, 2/3 위치에서)
        indices = [(num-1) // 3, (num-1) * 2 // 3]
        self.first_point = [repulsed_x[0], repulsed_y[0]]
        self.second_point = [repulsed_x[indices[0]], repulsed_y[indices[0]]]
        self.third_point = [repulsed_x[indices[1]], repulsed_y[indices[1]]]
        self.fourth_point = [repulsed_x[-1], repulsed_y[-1]]
        
        # Bezier 곡선 계산
        self.B = self.calc_curve(100)
        
        # 범위 포인트 추가
        # print(self.B)
        # Trajectory 메시지 생성 및 퍼블리시
        self.Trajectory_Generation()
        
        
        
        


    def Trajectory_Generation(self):
        i=0
        trajectory = Lane()
        for x, y in zip(self.B[0], self.B[1]):
            waypoint = Waypoint()
            waypoint.pose.pose.position.x = x
            waypoint.pose.pose.position.y = y
            # print(len(self.B[1]))
            if i < len(self.B[1])-1:
                quaternion=self.yaw_to_quaternion(math.atan2(self.B[1][i+1]-self.B[1][i],self.B[0][i+1]-self.B[0][i]))
            waypoint.pose.pose.orientation.x=quaternion[0]
            waypoint.pose.pose.orientation.y=quaternion[1]
            waypoint.pose.pose.orientation.z=quaternion[2]
            waypoint.pose.pose.orientation.w=quaternion[3]
            i+=1
            trajectory.waypoints.append(waypoint)
        print(len(trajectory.waypoints))
        self.overtaking_traj_pub.publish(trajectory)
        # Visualize the local path using markers
        marker_array = self.visualize_local_path(trajectory.waypoints)
        self.marker_pub.publish(marker_array)
        print("work1111")

        
    def calc_curve(self, granularity=100):
        'Calculate the cubic Bezier curve with the given granularity.'
        t_values = np.linspace(0, 1, granularity)
        
        # Precompute the coefficients for the cubic Bezier curve
        coeff1 = (1 - t_values) ** 3
        coeff2 = 3 * (1 - t_values) ** 2 * t_values
        coeff3 = 3 * (1 - t_values) * (t_values ** 2)
        coeff4 = t_values ** 3
        
        # Calculate x and y coordinates using vectorized operations
        B_x = (coeff1 * self.first_point[0] + 
            coeff2 * self.second_point[0] + 
            coeff3 * self.third_point[0] + 
            coeff4 * self.fourth_point[0])
        
        B_y = (coeff1 * self.first_point[1] + 
            coeff2 * self.second_point[1] + 
            coeff3 * self.third_point[1] + 
            coeff4 * self.fourth_point[1])

        return [B_x.tolist(), B_y.tolist()]
    

    
    
             
        
            

if __name__ == '__main__':
    try:
        Model_Predictive()
    except rospy.ROSInterruptException:
        pass