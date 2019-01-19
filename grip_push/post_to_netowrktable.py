import cv2
from networktables import NetworkTables
from .grip import GripPipeline


class NetworkTablePush:

    def __init__(self):

        print('Initializing NetworkTables')
        NetworkTables.setClientMode()
        NetworkTables.setIPAddress('10.60.09.2')
        NetworkTables.initialize()

    def extra_processing(self, pipeline):
        center_x_positions = []
        center_y_positions = []
        widths = []
        heights = []

    # Find the bounding boxes of the contours to get x, y, width, and height
        for contour in pipeline.filter_contours_output:
            x, y, w, h = cv2.boundingRect(contour)
            center_x_positions.append(x + w / 2)  # X and Y are coordinates of the top-left corner of the bounding box
            center_y_positions.append(y + h / 2)
            widths.append(w)
            heights.append(h)

        # Publish to the '/vision/red_areas' network table
        table = NetworkTables.getTable('')
        table.putNumberArray('x', center_x_positions)
        table.putNumberArray('y', center_y_positions)
        table.putNumberArray('width', widths)
        table.putNumberArray('height', heights)


if __name__ == '__main__':
    NetworkTablePush()
