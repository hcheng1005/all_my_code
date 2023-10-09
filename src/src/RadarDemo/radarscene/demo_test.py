'''
Author: CharlesCH hcheng1005@gmail.com
Date: 2023-02-20 21:00:50
LastEditors: CharlesCH hcheng1005@gmail.com
LastEditTime: 2023-10-09 21:46:09
FilePath: /radar_scenes/demo_test.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import os
from radar_scenes.sequence import Sequence
import numpy as np
import matplotlib.pyplot as plt 
import cv2

def main():

    plt.figure()

    # MODIFY THIS LINE AND INSERT PATH WHERE YOU STORED THE RADARSCENES DATASET
    path_to_dataset = "/media/charles/ShareDisk/00myDataSet/RadarScenes/RadarScenes"

    # Define the *.json file from which data should be loaded
    # some random sequence is chosen here.
    filename = os.path.join(path_to_dataset, "data", "sequence_2", "scenes.json")

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
    print("The second scene was measured by the radar sensor with the id {}".format(second_scene.sensor_id))

    # get the second measurement of the same sensor which also measured the first scene:
    second_measurement = sequence.next_scene_after(start_time, same_sensor=True)
    assert second_measurement.sensor_id == sequence.get_scene(start_time).sensor_id

    x_cc = []
    y_cc = []
    for idx, scene in enumerate(sequence.scenes()):
        if idx == 0:
            # check that start_time of the sequence is in fact identical to the timestamp of the first returned scene
            assert start_time == scene.timestamp
        radar_data = scene.radar_data

        if scene.sensor_id == 1:
            plt.clf()

            x_cc = np.append(x_cc, np.asarray(radar_data["x_cc"]))
            y_cc = np.append(y_cc, np.asarray(radar_data["y_cc"]))
            img = cv2.imread(scene.camera_image_name)
            cv2.imshow('img', img)
            plt.plot(np.asarray(y_cc), np.asarray(x_cc), 'b.')
            plt.axis([50, -50, -10, 200])
            plt.xticks(rotation=90) # 旋转90度
            plt.pause(0.01)

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
