from typing import Any
from cv_bridge import CvBridge
import cv2
import pandas as pd
import os


class DatasetCreator(object):
    def __init__(self, root_path: str) -> None:
        self.cv_bridge = CvBridge()
        self.buffer: dict = {}
        self.data_frame: pd.DataFrame = None
        self.root_path: str = root_path
        self.csv_path: str = None

        # Create the directory if it doesn't exist
        os.makedirs(self.root_path, exist_ok=True)

    def serialize_to_str(self, timestamp_str: str, column: str, data: Any) -> str:
        if column == "steering_angle":
            return f"{data:.4f}"
        elif "image" in column:
            return f"img/{column}_{timestamp_str}.jpg"
        else:
            raise NotImplementedError

    def write(self, timestamp_str: str, column: str, data: Any) -> None:
        self.buffer[timestamp_str] = {column: data}

    def get_buffer(self) -> dict:
        # Generate an iterator to iterate over the buffer and delete the read elements
        for timestamp_str in list(self.buffer.keys()):
            yield timestamp_str, self.buffer.pop(timestamp_str)

    def save_image(self, path: str, image: Any) -> None:
        cv2.imwrite(path, self.cv_bridge.imgmsg_to_cv2(image, "bgr8"))

    def start(self) -> None:
        self.csv_path = os.path.join(self.root_path, "dataset.csv")

        if os.path.exists(self.csv_path):
            self.data_frame = pd.read_csv(self.csv_path)
        else:
            self.data_frame = pd.DataFrame()

    def finish(self) -> None:
        if self.csv_path and not self.data_frame.empty:
            self.data_frame.to_csv(self.csv_path, index=False)
            self.csv_path = None
            self.data_frame = None

    def save_data_rows(self) -> None:
        for timestamp, data_dict in self.get_buffer():
            row_dict = {}
            for column, data in data_dict.items():
                row_dict[column] = self.serialize_to_str(timestamp, column, data)

                if 'image' in column:
                    self.save_image(os.path.join(self.root_path, row_dict[column]), data)

            new_row = pd.DataFrame([row_dict])
            self.data_frame = pd.concat([self.data_frame, new_row], ignore_index=True)

