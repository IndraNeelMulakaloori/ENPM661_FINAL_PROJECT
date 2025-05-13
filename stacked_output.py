import cv2
import numpy as np

def pad_frame(frame, target_size):
    return cv2.resize(frame, target_size)

# Load all videos
caps = [
    # cv2.VideoCapture("RRT_Traversal.mp4"),
    cv2.VideoCapture("RRT_star_Traversal.mp4"),
    cv2.VideoCapture("RRT_star_N_v2_Traversal.mp4"),
]

# Get target frame width and height (assumes all are same)
width = int(caps[0].get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(caps[0].get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = max([cap.get(cv2.CAP_PROP_FPS) for cap in caps])
fourcc = cv2.VideoWriter_fourcc(*'mp4v')

# Output writer
out = cv2.VideoWriter("stacked_output.mp4", fourcc, fps, (width, height * len(caps)))

# Track video status
last_frames = [None] * len(caps)
video_ended = [False] * len(caps)

while not all(video_ended):
    stacked_frames = []

    for i, cap in enumerate(caps):
        if not video_ended[i]:
            ret, frame = cap.read()
            if ret:
                last_frames[i] = pad_frame(frame, (width, height))
            else:
                video_ended[i] = True

        # If the video ended, reuse last valid frame
        if last_frames[i] is not None:
            stacked_frames.append(last_frames[i])
        else:
            # Fallback to black frame if something failed
            stacked_frames.append(np.zeros((height, width, 3), dtype=np.uint8))

    # Stack all vertically and write
    combined = np.vstack(stacked_frames)
    out.write(combined)

# Release everything
for cap in caps:
    cap.release()
out.release()
cv2.destroyAllWindows()
