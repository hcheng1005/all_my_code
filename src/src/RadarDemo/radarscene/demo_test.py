'''
Author: CharlesCH hcheng1005@gmail.com
Date: 2023-02-20 21:00:50
LastEditors: ChengHao hao.cheng@wuzheng.com
LastEditTime: 2023-10-20 11:53:19
FilePath: /all_my_code/src/src/RadarDemo/radarscene/demo_test.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import os
from radar_scenes.sequence import Sequence
import numpy as np
import matplotlib.pyplot as plt
import cv2

radar_mount = {"x": 3.663, "y": -0.873, "yaw": -1.48418552}


def radar_static_filter(radar_pc, ego_motion):
    # 构造转换矩阵
    radar_trans = np.array([np.cos(radar_mount['yaw']), -np.sin(radar_mount['yaw']), radar_mount['x'],
                            np.sin(radar_mount['yaw']), np.cos(
                                radar_mount['yaw']), radar_mount['y'],
                            0, 0, 1]).reshape([3, 3])

    radar_mount_angle = np.arctan(np.abs(radar_mount['x']/radar_mount['y']))
    radusial_ = np.sqrt(radar_mount['x']**2 + radar_mount['y']**2)
    
    # 雷达传感器线速度：有车辆旋转带来的线速度
    radar_vlinear = np.abs(radusial_*ego_motion[1])
    
    if ego_motion[1] < 0.0:
        radar_motion_v = np.sqrt(radar_vlinear**2+ego_motion[0]**2 - 2 * radar_vlinear * ego_motion[0] * np.cos(np.pi - radar_mount_angle))
    else:
        radar_motion_v = np.sqrt(radar_vlinear**2+ego_motion[0]**2 - 2 * radar_vlinear * ego_motion[0] * np.cos(radar_mount_angle))
   
    radar_motion_yaw = np.arcsin(radar_vlinear * np.sin(radar_mount_angle) / radar_motion_v)

    # 静态目标理论速度
    point_theory_v = np.cos((radar_mount['yaw'] + radar_motion_yaw + radar_pc[:,1])) * radar_motion_v
    
    radar_pc_vr_compensated = radar_pc[:,2] + point_theory_v
    print(radar_pc_vr_compensated - radar_pc[:,3])
    
def main():

    plt.figure()

    # MODIFY THIS LINE AND INSERT PATH WHERE YOU STORED THE RADARSCENES DATASET
    # path_to_dataset = "/media/charles/ShareDisk/00myDataSet/RadarScenes/RadarScenes"
    path_to_dataset = "/home/zdhjs-05/myGitHubCode/radar_scenes/RadarScenes"

    # Define the *.json file from which data should be loaded
    # some random sequence is chosen here.
    filename = os.path.join(path_to_dataset, "data",
                            "sequence_2", "scenes.json")

    if not os.path.exists(filename):
        print("Please modify this example so that it contains the correct path to the dataset on your machine.")
        return

    # create a Sequence object by passing the filename to the class method `from_json`
    sequence = Sequence.from_json(filename)

    print("The sequence contains {} different scenes".format(len(sequence)))

    # iterate over the individual radar measurements in this sequence and print the number of detections with RCS > 0
    start_time = sequence.first_timestamp

    # get the second scene in the sequence:
    second_scene = sequence.next_scene_after(start_time)
    print("The second scene was measured by the radar sensor with the id {}".format(
        second_scene.sensor_id))

    # get the second measurement of the same sensor which also measured the first scene:
    second_measurement = sequence.next_scene_after(
        start_time, same_sensor=True)
    assert second_measurement.sensor_id == sequence.get_scene(
        start_time).sensor_id

    x_cc = []
    y_cc = []
    for idx, scene in enumerate(sequence.scenes()):
        if idx == 0:
            # check that start_time of the sequence is in fact identical to the timestamp of the first returned scene
            assert start_time == scene.timestamp
        radar_data = scene.radar_data
        odometry_data = scene.odometry_data
        ego_motion = np.array([odometry_data['vx'], odometry_data['yaw_rate']])
        # print(odometry_data) #[timestamp, x_seq, y_seq, yaw_seq, vx, yaw_rate]

        if scene.sensor_id == 1:
            plt.clf()
            x_cc = np.append(x_cc, np.asarray(radar_data["x_cc"]))
            y_cc = np.append(y_cc, np.asarray(radar_data["y_cc"]))
            img = cv2.imread(scene.camera_image_name)
            # cv2.imshow('img', img)
            plt.plot(np.asarray(y_cc), np.asarray(x_cc), 'b.')
            plt.axis([50, -50, -10, 200])
            plt.xticks(rotation=90)  # 旋转90度
            plt.pause(0.01)

            R_ = np.asarray(radar_data["range_sc"])
            A_ = np.asarray(radar_data["azimuth_sc"])
            V_ = np.asarray(radar_data["vr"])
            vr_compensated = np.asarray(radar_data["vr_compensated"])
            
            radar_pc_info = np.concatenate([R_, A_, V_, vr_compensated]).reshape([4, -1]).transpose()
            radar_static_filter(radar_pc_info, ego_motion)

            x_cc = []
            y_cc = []

        if scene.sensor_id == 2 or scene.sensor_id == 3:
            if x_cc == []:
                x_cc = np.asarray(radar_data["x_cc"])
                y_cc = np.asarray(radar_data["y_cc"])
            else:
                x_cc = np.append(x_cc, np.asarray(radar_data["x_cc"]))
                y_cc = np.append(y_cc, np.asarray(radar_data["y_cc"]))

            # if y_cc == []:
            #     y_cc = np.asarray(radar_data["y_cc"])
            # else:
            #     y_cc = np.append(y_cc, np.asarray(radar_data["y_cc"]))

        # indices = np.where(radar_data["rcs"] > 0)[0]
        # print("Scene number {} at timestamp {} contains {} detections with RCS > 0".format(idx, scene.timestamp,
        #                                                                                    len(indices)))

    # iterate only over measurement from radar 1:
    for scene in sequence.scenes(sensor_id=1):
        assert scene.sensor_id == 1


if __name__ == '__main__':
    main()
