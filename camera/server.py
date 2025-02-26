#!/usr/bin/env python3

from numbers import Number
import sys
from typing import Tuple
import cv2
import math
import threading
import asyncio
import json
from camera import *
from vector2d import Vector2D
import itertools
import random
import angles
import time
from mqtt_client import MQTTClient

# --- COLORS ---
red = (0, 0, 255)
green = (0, 255, 0)
magenta = (255, 0, 255)
cyan = (255, 255, 0)
yellow = (50, 255, 255)
black = (0, 0, 0)
white = (255, 255, 255)

# -----------------------------------------
# 1) DATA CLASSES
# -----------------------------------------


class Tag:
    # -----------------------------------------
    # Represents a single detected ArUco tag in the image.
    # -----------------------------------------
    def __init__(self, id, raw_tag):
        self.id = id
        self.raw_tag = raw_tag
        self.corners = raw_tag.tolist()[0]

        # Store corners as Vector2D
        self.tl = Vector2D(int(self.corners[0][0]), int(self.corners[0][1]))  # top-left
        self.tr = Vector2D(
            int(self.corners[1][0]), int(self.corners[1][1])
        )  # top-right
        self.br = Vector2D(
            int(self.corners[2][0]), int(self.corners[2][1])
        )  # bottom-right
        self.bl = Vector2D(
            int(self.corners[3][0]), int(self.corners[3][1])
        )  # bottom-left

        # Corners:  (tl)--------(tr)
        #           |             |
        #           |     (C)     |    <-- (C) is the center
        #           |             |
        #           (bl)--------(br)
        # Calculate centre of the tag

        self.centre = Vector2D(
            int((self.tl.x + self.tr.x + self.br.x + self.bl.x) / 4),
            int((self.tl.y + self.tr.y + self.br.y + self.bl.y) / 4),
        )

        # Front midpoint: (front)
        #           (tl)--------(tr)
        #             ^-- midpoint of this edge
        # Midpoint of the top edge ("front" of the tag)
        self.front = Vector2D(
            int((self.tl.x + self.tr.x) / 2), int((self.tl.y + self.tr.y) / 2)
        )

        # Orientation (radians) measured from center to the top edge  Forward vector
        self.forward = math.atan2(
            self.front.y - self.centre.y, self.front.x - self.centre.x
        )
        self.angle = math.degrees(self.forward)


class Robot:
    # """
    # Represents a robot identified by an ArUco tag in the environment.
    # """
    def __init__(self, tag, position):
        self.tag = tag
        self.id = tag.id
        self.position = position
        self.orientation = tag.angle
        self.sensor_range = 0.3  # 30cm sensing radius
        self.neighbours = {}
        self.tasks = {}


class SensorReading:
    def __init__(self, range, bearing, orientation=0, workers=0):
        self.range = range
        self.bearing = bearing
        self.orientation = orientation
        self.workers = workers


class Task:
    # """
    # Represents a task that can be done by robots within the environment.
    # """
    def __init__(self, id, workers, position, radius, time_limit):
        self.id = id
        self.workers = workers
        self.position = position
        self.radius = radius
        self.time_limit = time_limit
        self.counter = time_limit
        self.completed = False
        self.failed = False
        self.start_time = time.time()
        self.arrival_time = time.time()
        self.completing = False


# -----------------------------------------
# 2) MAIN TRACKER CLASS
# -----------------------------------------


class Tracker:
    # """
    # The main tracker handles:
    #   - Camera input (via a Camera() object)
    #   - Detecting ArUco markers
    #   - Converting pixel coordinates to real-world coordinates
    #   - Tracking and drawing the current state onto the live camera frame
    # """
    def __init__(self):
        # Initialize camera
        self.camera = Camera()

        # Calibration parameters
        self.calibrated = False
        self.num_corner_tags = 0

        # Pixel boundaries (updated after corner detection)
        self.min_x = 0
        self.min_y = 0
        self.max_x = 0
        self.max_y = 0

        # Real-world center of the environment
        self.centre = Vector2D(0, 0)

        # Known distance in real-world (meters) between corner tags
        self.corner_distance_metres = 0.27
        self.corner_distance_pixels = 0
        self.scale_factor = 0

        # Dictionaries for robots and tasks
        self.robots = {}
        self.tasks = {}
        self.task_counter = 0
        self.score = 0

        self.mqtt_client = MQTTClient("test.mosquitto.org", 1883)

    def detect_markers(self, image):
        # """
        # Detect ArUco markers in the given frame using OpenCV ArUco.
        # Returns (raw_tags, tag_ids, rejected).
        # """
        aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        aruco_parameters = cv2.aruco.DetectorParameters()
        aruco_detector = cv2.aruco.ArucoDetector(aruco_dictionary, aruco_parameters)
        return aruco_detector.detectMarkers(image)

    # This function:
    # Iterates over all detected tags.
    # Calibration with corner tags (ID=0):
    # If it’s the first corner tag encountered (self.num_corner_tags == 0), store its pixel coordinates as initial min_x, max_x, etc.
    # If it’s the second corner tag, update min_x, max_x, min_y, max_y to define the bounding rectangle in pixel space.
    # Compute the distance (in pixels) between the two corner tags, then compute scale_factor = pixels / distance_in_meters.
    # Compute a coordinate system center (centre) in meters, from the midpoint of these corner tags in pixel coordinates (converted using the scale_factor).
    # Mark calibrated = True.
    def process_tags(self, tag_ids, raw_tags):
        for id, raw_tag in zip(tag_ids, raw_tags):
            tag = Tag(id, raw_tag)
            if tag.id == 0:  # Reserved tag ID for corners
                if self.num_corner_tags == 0:  # Record the first corner tag detected
                    self.min_x = tag.centre.x
                    self.max_x = tag.centre.x
                    self.min_y = tag.centre.y
                    self.max_y = tag.centre.y
                else:  # Set min/max boundaries of arena based on second corner tag detected
                    if tag.centre.x < self.min_x:
                        self.min_x = tag.centre.x
                    if tag.centre.x > self.max_x:
                        self.max_x = tag.centre.x
                    if tag.centre.y < self.min_y:
                        self.min_y = tag.centre.y
                    if tag.centre.y > self.max_y:
                        self.max_y = tag.centre.y

                    self.corner_distance_pixels = math.dist(
                        [self.min_x, self.min_y], [self.max_x, self.max_y]
                    )  # Euclidean distance between corner tags in pixels
                    self.scale_factor = (
                        self.corner_distance_pixels / self.corner_distance_metres
                    )
                    x = (
                        (self.max_x - self.min_x) / 2
                    ) / self.scale_factor  # Convert to metres
                    y = (
                        (self.max_y - self.min_y) / 2
                    ) / self.scale_factor  # Convert to metres
                    self.centre = Vector2D(x, y)
                    self.calibrated = True
                self.num_corner_tags = self.num_corner_tags + 1

        # After calibrating, loop through the tags again to create Robot objects for any non-corner tags.
        # position = Vector2D(tag.centre.x / scale_factor, tag.centre.y / scale_factor): Convert from pixel coordinates to real-world meters.
        # Add the Robot to self.robots dictionary.

        for id, raw_tag in zip(tag_ids, raw_tags):
            tag = Tag(id, raw_tag)

            if self.calibrated and tag.id != 0:
                # Reserved tag ID for corners
                position = Vector2D(
                    tag.centre.x / self.scale_factor,
                    tag.centre.y / self.scale_factor,
                )  # Convert pixel coordinates to metres
                self.robots[id] = Robot(tag, position)

    def draw_frame(self, image):
        window_name = "Winter School tutorial Demo"

        cv2.imshow(window_name, image)

        if cv2.waitKey(1) == ord("q"):
            sys.exit()

    def run(self):
        # """
        # Main loop:
        #   - Grab frame from camera
        #   - Detect all markers
        #   - Build Tag objects
        #   - Calibrate if 2 corner tags found
        #   - Create Robot objects (if calibrated)
        #   - Draw bounding rectangle, robots, text
        # """
        while True:
            self.calibrated = False
            self.num_corner_tags = 0
            self.robots = {}

            # 1) Grab camera frame
            image = self.camera.get_frame()
            overlay = image.copy()

            # 2) Detect any ArUco markers
            raw_tags, tag_ids, rejected = self.detect_markers(image)

            # If no markers, just display the frame
            if tag_ids is None or len(tag_ids) == 0:
                self.draw_frame(image)
                continue

            # Flatten and convert numpy IDs to integer
            tag_ids = list(itertools.chain(*tag_ids))
            tag_ids = [int(id) for id in tag_ids]  # Convert from numpy.int32 to int

            # Process raw ArUco output and assign self.robots
            self.process_tags(tag_ids, raw_tags)

            # If not calibrated (haven't found two corner tags), just show the frame
            if not self.calibrated:
                self.draw_frame(image)
                continue

            # Draw boundary of virtual environment based on corner tag positions
            cv2.rectangle(
                image,
                (self.min_x, self.min_y),
                (self.max_x, self.max_y),
                green,
                1,
                lineType=cv2.LINE_AA,
            )

            # Print & send location information
            for id, robot in self.robots.items():
                robot.position.x = round(robot.position.x, 6)
                robot.position.y = round(robot.position.y, 6)
                robot.orientation = round(robot.orientation, 6)
                self.mqtt_client.publish(
                    "eums-2025/%d/location" % id,
                    json.dumps(
                        {
                            "x": robot.position.x,
                            "y": robot.position.y,
                            "orientation": robot.orientation,
                        }
                    ),
                )
                print(
                    f"ID: {id}, x = {robot.position.x}, y = {robot.position.y}, orientation = {robot.orientation}"
                )

            # Draw each robot
            for robot_id, robot in self.robots.items():
                tag = robot.tag

                # Draw tag border lines
                pts = [
                    (tag.tl.x, tag.tl.y),
                    (tag.tr.x, tag.tr.y),
                    (tag.br.x, tag.br.y),
                    (tag.bl.x, tag.bl.y),
                ]
                for i in range(4):
                    cv2.line(
                        image, pts[i], pts[(i + 1) % 4], green, 2, lineType=cv2.LINE_AA
                    )

                # Draw circle on centre point
                cv2.circle(
                    image,
                    (tag.centre.x, tag.centre.y),
                    5,
                    red,
                    -1,
                    lineType=cv2.LINE_AA,
                )

                # Draw line from centre point to front of tag    # Draw orientation line from center to front
                forward_point = ((tag.front - tag.centre) * 2) + tag.centre
                cv2.line(
                    image,
                    (tag.centre.x, tag.centre.y),
                    (forward_point.x, forward_point.y),
                    black,
                    10,
                    lineType=cv2.LINE_AA,
                )
                cv2.line(
                    image,
                    (tag.centre.x, tag.centre.y),
                    (forward_point.x, forward_point.y),
                    green,
                    3,
                    lineType=cv2.LINE_AA,
                )

                # Draw tag ID text
                text = str(tag.id)
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 1.5
                thickness = 4
                textsize = cv2.getTextSize(text, font, font_scale, thickness)[0]
                text_x = tag.centre.x - textsize[0] // 2
                text_y = tag.centre.y + textsize[1] // 2

                # Text shadow
                cv2.putText(
                    image,
                    text,
                    (text_x, text_y),
                    font,
                    font_scale,
                    black,
                    thickness * 3,
                    cv2.LINE_AA,
                )
                # Text foreground
                cv2.putText(
                    image,
                    text,
                    (text_x, text_y),
                    font,
                    font_scale,
                    white,
                    thickness,
                    cv2.LINE_AA,
                )

                def draw_text(
                    image,
                    text,
                    position,
                    font_scale=2,
                    thickness=5,
                    text_color=(255, 255, 255),
                    outline_color=(0, 0, 0),
                ):
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(
                        image,
                        text,
                        position,
                        font,
                        font_scale,
                        outline_color,
                        thickness * 3,
                        cv2.LINE_AA,
                    )  # Outline
                    cv2.putText(
                        image,
                        text,
                        position,
                        font,
                        font_scale,
                        text_color,
                        thickness,
                        cv2.LINE_AA,
                    )  # Text

                # TEMP: Draw x, y, angle
                if id == 1:
                    texts = {
                        "x": round(robot.position.x, 6),
                        "y": round(robot.position.y, 6),
                        "theta": round(robot.orientation, 2),
                    }

                    y_height = 0

                    for field, value in texts.items():
                        text = f"{field}: {value}"
                        text_size = cv2.getTextSize(
                            text, cv2.FONT_HERSHEY_SIMPLEX, 2, 5
                        )[0]
                        line_height = text_size[1] + 20
                        y_height += line_height
                        position = (10, y_height)

                        # Draw text on the image and overlay
                        draw_text(image, text, position)
                        draw_text(overlay, text, position)

            # Transparency for overlaid augments
            alpha = 0.3
            image = cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0)

            time.sleep(0.5)

            self.draw_frame(image)


# TODO: Handle Ctrl+C signals
if __name__ == "__main__":

    # Start tracker
    global tracker
    tracker = Tracker()
    tracker.run()
