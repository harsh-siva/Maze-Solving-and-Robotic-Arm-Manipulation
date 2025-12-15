import cv2
import numpy as np
from matplotlib import pyplot as plt 
import csv

def displayImg(imgList, labelList = None):
        
        if labelList is not None:
            if len(labelList) == len(imgList):

                for i,j in zip(imgList, labelList): 
                     plt.imshow(cv2.cvtColor(i, cv2.COLOR_BGR2RGB))
                     plt.axis('off')
                     plt.title(j)
                     plt.show()

            else:
                print("ERROR: List of images and list of labels is not of the same size.")

        else:

                for i in imgList:
                    plt.imshow(cv2.cvtColor(1, cv2.COLOR_BGR2RGB))
                    plt.axis('off')
                    plt.show()

def capture_and_crop():

    # Open the camera
    cap = cv2.VideoCapture(2)  
    if not cap.isOpened():
        raise Exception("Camera not accessible!")
    
    print("Adjust the camera and press 'c' to capture the image of the maze.")
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        cv2.imshow("Camera Feed", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            captured_image = frame
            break

    cap.release()
    cv2.destroyAllWindows()

    # captured_image = cv2.imread('captured.jpeg') 

    print("Draw a rectangle to crop the maze.")
    r = cv2.selectROI("Select Maze Region", captured_image)
    cropped_maze = captured_image[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]
    cv2.destroyAllWindows()

    # Save the image
    output_path = 'cropped.png'
    cv2.imwrite(output_path, cropped_maze)
    print(f"Maze solution image saved to {output_path}")


    return cropped_maze


def solve_maze(cropped_img):
    orgimg = cv2.imread(cropped_img,0)
    # orgimg = cropped_img
    img = orgimg.copy()
    

    ret, binary_image = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    

    contours , hierarchy = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    path = np.zeros(binary_image.shape, np.uint8)
    cv2.drawContours(path, contours, 0, (255,255,255), 10)

    #WHITE SPACE EXPANDS
    kernel = np.ones((10,10),np.uint8)
    dilated = cv2.dilate(path, kernel, iterations=11)
    eroded = cv2.erode(dilated, kernel, iterations = 1)
    diff = cv2.absdiff(eroded, dilated)
    img = cv2.cvtColor(img,cv2.COLOR_GRAY2RGB)
    img[diff==255] = (0,255,0)
    maze_solution_green = img

    # Save the image
    output_path = 'maze_solution_green.png'
    cv2.imwrite(output_path, maze_solution_green)
    print(f"Maze solution image saved to {output_path}")

def extract_limited_waypoints(img, maze_dimensions, max_waypoints):
    green_mask = (img[:, :, 0] == 0) & (img[:, :, 1] == 255) & (img[:, :, 2] == 0)
    
    # Get pixel coordinates of green path
    path_pixels = np.column_stack(np.where(green_mask))

    # Normalize to the maze dimensions (assuming top-left (0,0) and bottom-right (16,16))
    maze_height, maze_width = img.shape[:2]
    normalized_waypoints = [
        (
            (x / maze_width) * maze_dimensions[0],
            (y / maze_height) * maze_dimensions[1]
        )
        for y, x in path_pixels
    ]

    # Restrict the waypoints to no more than max_waypoints
    if len(normalized_waypoints) > max_waypoints:
        step = len(normalized_waypoints) // max_waypoints
        limited_waypoints = normalized_waypoints[::step][:max_waypoints]
    else:
        limited_waypoints = normalized_waypoints
    
    limited_waypoints_last = limited_waypoints[-1]
    print(limited_waypoints_last)
    limited_waypoints_1 = limited_waypoints + [(limited_waypoints_last[0],maze_dimensions[1])]

    return limited_waypoints_1

def display_waypoints_on_image(img, waypoints, maze_dimensions, point_color=(0, 0, 255), point_radius=5):
    """
    Displays waypoints on the given image using the provided displayImg function.
    - img: The image on which waypoints are to be displayed.
    - waypoints: List of waypoints (x, y) in normalized coordinates.
    - point_color: Color of the waypoints (default: red).
    - point_radius: Radius of each waypoint point.
    """
    img_copy = img.copy()

    # Convert normalized waypoints to pixel coordinates
    height, width = img.shape[:2]
    for x, y in waypoints:
        pixel_x = int((x / maze_dimensions[0]) * width)
        pixel_y = int((y / maze_dimensions[1]) * height)

        # Draw circle at each waypoint
        cv2.circle(img_copy, (pixel_x, pixel_y), point_radius, point_color, -1)

    # Use your displayImg function to show the image
    displayImg([img_copy], ["Waypoints on Image"])

def map_coordinates_to_robot(waypoints, corners, maze_dimensions):
    """
    Maps maze waypoints to robot coordinates using linear interpolation.
    
    Args:
    - waypoints: List of waypoints in the maze coordinate system [(x, y), ...].
    - corners: Four robot coordinate corners in the order:
               [top-left, top-right, bottom-left, bottom-right].
    - maze_dimensions: Tuple representing maze grid size (width, height).

    Returns:
    - robot_trajectory: List of waypoints in the robot coordinate system.
    """
    top_left, top_right, bottom_left, bottom_right = corners
    maze_width, maze_height = maze_dimensions

    # Linear interpolation for x and y coordinates
    def interpolate(value, src_min, src_max, dst_min, dst_max):
        return dst_min + ((value - src_min) / (src_max - src_min)) * (dst_max - dst_min)

    robot_trajectory = []
    for point in waypoints:
        x_maze, y_maze = point  # Maze coordinates
        x_robot = interpolate(x_maze, 0, maze_width - 1, top_left[0], top_right[0])
        y_robot = interpolate(y_maze, 0, maze_height - 1, top_left[1], bottom_left[1])
        robot_trajectory.append((x_robot, y_robot, 230))

    return robot_trajectory

def save_trajectory_to_csv(robot_trajectory, filename='robot_trajectory.csv'):
    """
    Save robot trajectory points to a CSV file.
    
    Args:
    - robot_trajectory: List of tuples containing (x, y, z) coordinates
    - filename: Name of the output CSV file
    """
    
    # Write the trajectory points to the CSV file
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        
        # Write each trajectory point
        for point in robot_trajectory:
            writer.writerow(point)
    
    print(f"Trajectory points saved to {filename}")


if __name__ == "__main__":
    # Maze and robot configuration
    maze_dimensions = (6, 6)  # Maze dimensions in logical units (e.g., 6x6 grid)
    max_waypoints = 15  # Limit on the number of waypoints

    cropped_maze = capture_and_crop()

    cropped = "cropped.png"
    solve_maze(cropped)

  
    solution_image = cv2.imread('maze_solution_green.png')
    limited_waypoints = extract_limited_waypoints(solution_image, maze_dimensions=(6,6), max_waypoints=50)

    # Display waypoints on the solution image
    display_waypoints_on_image(solution_image, limited_waypoints, maze_dimensions=(6,6))

    #TAKE MAZE CORNER COORDINATES MANUALLY TO CALIBRATE WORLD COORDINATES WITH ROBOT COORDINATES
    robot_corners = [(409.0,270.0,230),(267.0,270.0,230),(409.0,414.0,230),(267.0,414.0,230)]
    
    robot_trajectory = map_coordinates_to_robot(limited_waypoints, robot_corners, maze_dimensions=(6,6))

    Return_list = robot_trajectory

    
    # Add this line after generating robot_trajectory
    save_trajectory_to_csv(robot_trajectory)

    # Display the mapped robot trajectory
    print("--- Robot Trajectory (Robot Coordinate System) ---")
    for i, point in enumerate(robot_trajectory, start=1):
        print(f"Waypoint {i}: {point}")
