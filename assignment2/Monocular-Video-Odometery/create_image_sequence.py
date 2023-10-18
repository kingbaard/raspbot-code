import cv2
import os
import argparse
import numpy as np

# Create an argument parser
parser = argparse.ArgumentParser(description="Convert a video into an image sequence")

# Add the video path argument
parser.add_argument("video_path", help="Path to the input video file")

# Add the output directory argument
parser.add_argument("output_directory", help="Directory to save the image sequence")

# Parse the command-line arguments
args = parser.parse_args()

# Open the video file
cap = cv2.VideoCapture(args.video_path)

# Create the output directory if it doesn't exist
os.makedirs(args.output_directory, exist_ok=True)

frame_count = 0

while True:
    ret, frame = cap.read()
    # ret, frame = cap.read()
    # ret, frame = cap.read()
    # ret, frame = cap.read()
    # ret, frame = cap.read()
    # ret, frame = cap.read()
    # ret, frame = cap.read()
    # ret, frame = cap.read()
    # ret, frame = cap.read()
    # ret, frame = cap.read()
    # Get every 10th frame

    if not ret:
        break

    # Save each frame as an image
    frame_filename = f"{frame_count:06d}.png"  # You can adjust the filename format
    frame_path = os.path.join(args.output_directory, frame_filename)

    # Sharpen image
    kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
    frame = cv2.filter2D(frame, -1, kernel)

    cv2.imwrite(frame_path, frame)

    frame_count += 1

# Release the video capture object
cap.release()

print(f"Saved {frame_count} frames to {args.output_directory}")
