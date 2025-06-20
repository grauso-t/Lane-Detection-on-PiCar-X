import cv2
import numpy as np
import math
from picamera2 import Picamera2
from time import sleep
from picarx import Picarx

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()
sleep(2)

px = Picarx()
px.forward(0)

lane_width = 280  # Lane width in pixels
previous_angle = 0  # Previous steering angle
frame_center = 150  # Fixed image center
dst_w, dst_h = 300, 200  # Transformed image dimensions

# Angle control parameters
SPEED = 1 # Movement speed (1-100)
MIN_ANGLE = -40.0  # Minimum angle in degrees
MAX_ANGLE = 40.0   # Maximum angle in degrees
ANGLE_SMOOTHING = 0.2  # Angle smoothing factor (0-1)
SINGLE_LINE_OFFSET = 0  # pixel offset towards road center

# Parameter for dynamic center position (smaller = higher up)
CENTER_Y_RATIO = 0.22  # 0.3 means at 30% of height (higher up compared to 0.5)

# Parameter for minimum distance from center (in pixels)
MIN_DISTANCE_FROM_CENTER = 10  # Minimum distance a line must have from center

def bird_eye_transform(frame):
    """Apply bird's eye view transformation"""
    h, w = frame.shape[:2]
    
    # Define region of interest (ROI) in the lower part of the frame
    roi_top, roi_bottom = int(h * 0.6), h
    
    # Source points for perspective transformation
    src = np.float32([
        [0, roi_top], 
        [w, roi_top], 
        [w, roi_bottom], 
        [0, roi_bottom]
    ])
    
    # Destination points
    dst = np.float32([
        [0, 0], 
        [dst_w, 0], 
        [dst_w, dst_h], 
        [0, dst_h]
    ])
    
    # Calculate and apply transformation
    matrix = cv2.getPerspectiveTransform(src, dst)
    bird_eye = cv2.warpPerspective(frame, matrix, (dst_w, dst_h))
    
    return bird_eye

def detect_lane_lines(bird_eye):
    """Detect lane lines"""
    # Convert to HSV
    hsv = cv2.cvtColor(bird_eye, cv2.COLOR_BGR2HSV)
    
    # Masks for white and yellow colors
    mask_white = cv2.inRange(hsv, np.array([0, 0, 200]), np.array([180, 30, 255]))
    mask_yellow = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([30, 255, 255]))
    
    # Combine masks
    mask = cv2.bitwise_or(mask_white, mask_yellow)
    
    # Apply filters
    blurred = cv2.GaussianBlur(mask, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    
    # Line detection with HoughLinesP
    lines = cv2.HoughLinesP(
        edges, 1, np.pi / 180, 
        threshold=50, 
        minLineLength=20, 
        maxLineGap=30
    )
    
    return lines, edges, mask

def separate_lines(lines):
    """Separate lines into left and right"""
    if lines is None:
        return None, None
    
    left_lines = []
    right_lines = []
    
    for line in lines:
        x1, y1, x2, y2 = line[0]
        
        # Calculate slope
        if x2 - x1 != 0:
            slope = (y2 - y1) / (x2 - x1)
            
            # Classify line based on slope and position
            if slope < 0 and x1 < frame_center and x2 < frame_center:
                left_lines.append(line[0])
            elif slope > 0 and x1 > frame_center and x2 > frame_center:
                right_lines.append(line[0])
    
    return left_lines, right_lines

def average_lines(lines):
    """Calculate average line from a group of lines"""
    if not lines:
        return None
    
    x_coords = []
    y_coords = []
    
    for line in lines:
        x1, y1, x2, y2 = line
        x_coords.extend([x1, x2])
        y_coords.extend([y1, y2])
    
    if len(x_coords) < 2:
        return None
    
    # Linear regression
    poly = np.polyfit(y_coords, x_coords, 1)
    
    # Calculate line points
    y1 = dst_h
    y2 = int(dst_h * 0.6)
    x1 = int(poly[0] * y1 + poly[1])
    x2 = int(poly[0] * y2 + poly[1])
    
    return [x1, y1, x2, y2]

def get_line_x_at_y(line, target_y):
    """Calculate x coordinate of a line at a specific y coordinate"""
    if line is None:
        return None
    
    x1, y1, x2, y2 = line
    
    # Avoid division by zero
    if y2 - y1 == 0:
        return x1
    
    # Linear interpolation to find x at target y coordinate
    t = (target_y - y1) / (y2 - y1)
    x = x1 + t * (x2 - x1)
    
    return x

def validate_lines(left_line, right_line):
    """Validate detected lines with position and distance from center checks"""
    valid_left = False
    valid_right = False
    
    # Calculate y coordinate for position check (same as dynamic center)
    center_y = int(dst_h * CENTER_Y_RATIO)
    
    if left_line is not None:
        x1, y1, x2, y2 = left_line
        line_length = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        
        # Check minimum length
        if line_length >= 20:
            # Calculate line x position at dynamic center
            line_x_at_center = get_line_x_at_y(left_line, center_y)
            
            # Left line must be:
            # 1. To the left of center
            # 2. Sufficiently distant from center
            if (line_x_at_center is not None and 
                line_x_at_center < frame_center and 
                abs(line_x_at_center - frame_center) >= MIN_DISTANCE_FROM_CENTER):
                valid_left = True
    
    if right_line is not None:
        x1, y1, x2, y2 = right_line
        line_length = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        
        # Check minimum length
        if line_length >= 20:
            # Calculate line x position at dynamic center
            line_x_at_center = get_line_x_at_y(right_line, center_y)
            
            # Right line must be:
            # 1. To the right of center
            # 2. Sufficiently distant from center
            if (line_x_at_center is not None and 
                line_x_at_center > frame_center and 
                abs(line_x_at_center - frame_center) >= MIN_DISTANCE_FROM_CENTER):
                valid_right = True
    
    return valid_left, valid_right

def calculate_lane_center_and_angle(left_line, right_line, valid_left, valid_right):
    """Calculate lane center and steering angle"""
    global previous_angle
    
    lane_center = None
    steering_angle = previous_angle  # Keep previous angle as default
    
    # Calculate y coordinate for dynamic center (higher up)
    center_y = int(dst_h * CENTER_Y_RATIO)
    
    # ADDITIONAL CHECK: Verify lines are in correct position
    # and sufficiently distant from center
    position_check_passed = True
    
    if valid_left:
        left_x = get_line_x_at_y(left_line, center_y)
        if left_x is not None:
            # Check if line is in wrong position or too close to center
            if left_x >= frame_center:
                # Left line is to the right of center - detection error
                valid_left = False
                position_check_passed = False
                print("WARNING: Left line detected to the right of center - ignored")
            elif abs(left_x - frame_center) < MIN_DISTANCE_FROM_CENTER:
                # Left line too close to center
                valid_left = False
                position_check_passed = False
                print(f"WARNING: Left line too close to center ({abs(left_x - frame_center):.1f}px < {MIN_DISTANCE_FROM_CENTER}px) - ignored")
    
    if valid_right:
        right_x = get_line_x_at_y(right_line, center_y)
        if right_x is not None:
            # Check if line is in wrong position or too close to center
            if right_x <= frame_center:
                # Right line is to the left of center - detection error
                valid_right = False
                position_check_passed = False
                print("WARNING: Right line detected to the left of center - ignored")
            elif abs(right_x - frame_center) < MIN_DISTANCE_FROM_CENTER:
                # Right line too close to center
                valid_right = False
                position_check_passed = False
                print(f"WARNING: Right line too close to center ({abs(right_x - frame_center):.1f}px < {MIN_DISTANCE_FROM_CENTER}px) - ignored")
    
    # If position check didn't pass, keep previous angle
    if not position_check_passed:
        print(f"Keeping previous angle: {previous_angle:.1f}°")
        return lane_center, previous_angle, center_y
    
    # If we have both valid lines
    if valid_left and valid_right:
        left_x = get_line_x_at_y(left_line, center_y)
        right_x = get_line_x_at_y(right_line, center_y)
        
        if left_x is not None and right_x is not None:
            lane_center = (left_x + right_x) / 2
    
    # If we only have left line - add offset to the right
    elif valid_left and not valid_right:
        left_x = get_line_x_at_y(left_line, center_y)
        if left_x is not None:
            # Estimated lane center + offset to the right for safety
            lane_center = left_x + lane_width / 2 + SINGLE_LINE_OFFSET
    
    # If we only have right line - add offset to the left
    elif valid_right and not valid_left:
        right_x = get_line_x_at_y(right_line, center_y)
        if right_x is not None:
            # Estimated lane center + offset to the left for safety
            lane_center = right_x - lane_width / 2 - SINGLE_LINE_OFFSET
    
    # Calculate steering angle if we have a valid center
    if lane_center is not None:
        # Distance from image center
        center_offset = lane_center - frame_center
        
        # Calculate angle based on offset
        # Use a more aggressive function to reach ±45° limits
        max_offset = dst_w / 2  # Maximum possible offset
        
        # Normalize offset between -1 and 1
        normalized_offset = center_offset / max_offset
        
        # Apply function to increase sensitivity
        # Use modified sigmoid function to get full range
        if abs(normalized_offset) > 0.1:  # Threshold to avoid jitter at center
            # Exponential function to increase sensitivity
            sign = 1 if normalized_offset > 0 else -1
            abs_offset = abs(normalized_offset)
            
            # Map offset with more aggressive curve
            if abs_offset < 0.5:
                mapped_offset = abs_offset * 1.5  # Linear increase for small offsets
            else:
                mapped_offset = 0.75 + (abs_offset - 0.5) * 1.5  # Accelerated increase
            
            mapped_offset = min(mapped_offset, 1.0)  # Limit to 1
            steering_angle = sign * mapped_offset * MAX_ANGLE
        else:
            steering_angle = 0  # Dead zone for stability
        
        # Apply hard limits
        steering_angle = np.clip(steering_angle, MIN_ANGLE, MAX_ANGLE)
        
        # Apply smoothing to reduce oscillations
        steering_angle = (ANGLE_SMOOTHING * previous_angle + 
                         (1 - ANGLE_SMOOTHING) * steering_angle)
        
        # Update previous angle
        previous_angle = steering_angle
    else:
        # If we don't have a valid center, keep previous angle
        print(f"No valid center - keeping previous angle: {previous_angle:.1f}°")
    
    return lane_center, steering_angle, center_y

def draw_lanes(img, left_line, right_line, valid_left, valid_right, lane_center, center_y):
    """Draw lane lines on image"""
    line_img = img.copy()
    
    # Draw left line
    if valid_left and left_line is not None:
        cv2.line(line_img, (left_line[0], left_line[1]), 
                (left_line[2], left_line[3]), (0, 255, 0), 3)
        cv2.putText(line_img, "L", (left_line[0]-20, left_line[1]+20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # Draw right line
    if valid_right and right_line is not None:
        cv2.line(line_img, (right_line[0], right_line[1]), 
                (right_line[2], right_line[3]), (0, 255, 0), 3)
        cv2.putText(line_img, "R", (right_line[0]+10, right_line[1]+20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # Draw lane center (now higher up)
    if lane_center is not None:
        cv2.circle(line_img, (int(lane_center), center_y), 8, (255, 0, 255), -1)
        cv2.putText(line_img, "CENTER", (int(lane_center)-30, center_y-15), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
    
    # Draw horizontal line to show dynamic center height
    cv2.line(line_img, (0, center_y), (dst_w, center_y), (255, 0, 255), 1)
    
    # Draw fixed image center
    cv2.line(line_img, (frame_center, 0), (frame_center, dst_h), 
            (0, 0, 255), 2)
    cv2.putText(line_img, "IMG CENTER", (frame_center-45, 20), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    return line_img

def draw_steering_indicator(img, steering_angle):
    """Draw visual steering angle indicator"""
    # Indicator position
    center_x, center_y = dst_w - 50, 50
    radius = 30
    
    # Draw base circle
    cv2.circle(img, (center_x, center_y), radius, (100, 100, 100), 2)
    
    # Calculate arrow position based on angle
    angle_rad = math.radians(steering_angle)
    end_x = int(center_x + radius * 0.8 * math.sin(angle_rad))
    end_y = int(center_y - radius * 0.8 * math.cos(angle_rad))
    
    # Color based on angle intensity
    intensity = abs(steering_angle) / MAX_ANGLE
    if intensity < 0.3:
        color = (0, 255, 0)  # Green for small angles
    elif intensity < 0.7:
        color = (0, 255, 255)  # Yellow for medium angles
    else:
        color = (0, 0, 255)  # Red for large angles
    
    # Draw arrow
    cv2.arrowedLine(img, (center_x, center_y), (end_x, end_y), color, 3)
    
    # Add text with angle
    cv2.putText(img, f"{steering_angle:.1f}°", 
               (center_x - 25, center_y + radius + 15), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    
    return img

def add_info_overlay(img, steering_angle, lane_center, valid_left, valid_right, center_y):
    """Add text information overlay to image"""
    info_img = img.copy()
    
    # Status information
    y_offset = 30
    # Color for angle based on limits
    angle_color = (0, 255, 0) if -15 <= steering_angle <= 15 else (0, 255, 255) if -30 <= steering_angle <= 30 else (0, 0, 255)
    cv2.putText(info_img, f"Steering: {steering_angle:.1f}°", 
               (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, angle_color, 2)
    
    y_offset += 25
    left_status = "OK" if valid_left else "NO"
    cv2.putText(info_img, f"Left Line: {left_status}", 
               (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 
               (0, 255, 0) if valid_left else (0, 0, 255), 2)
    
    y_offset += 25
    right_status = "OK" if valid_right else "NO"
    cv2.putText(info_img, f"Right Line: {right_status}", 
               (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 
               (0, 255, 0) if valid_right else (0, 0, 255), 2)
    
    y_offset += 25
    if lane_center is not None:
        offset = lane_center - frame_center
        cv2.putText(info_img, f"Offset: {offset:.1f}px", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    y_offset += 25
    # Show dynamic center height
    cv2.putText(info_img, f"Center Y: {center_y}px ({CENTER_Y_RATIO:.1f})", 
               (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
    
    y_offset += 25
    # Show minimum distance from center
    cv2.putText(info_img, f"Min Dist: {MIN_DISTANCE_FROM_CENTER}px", 
               (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    
    # Add visual steering indicator
    info_img = draw_steering_indicator(info_img, steering_angle)
    
    return info_img

def process_frame(frame):
    """Process a single frame"""
    # Bird's eye view transformation
    bird_eye = bird_eye_transform(frame)
    
    # Line detection
    lines, edges, mask = detect_lane_lines(bird_eye)
    
    # Separate left/right lines
    left_lines, right_lines = separate_lines(lines)
    
    # Calculate average lines
    left_line = average_lines(left_lines)
    right_line = average_lines(right_lines)
    
    # Validate lines
    valid_left, valid_right = validate_lines(left_line, right_line)
    
    # Calculate lane center and steering angle
    lane_center, steering_angle, center_y = calculate_lane_center_and_angle(
        left_line, right_line, valid_left, valid_right)
    
    # Create output images
    lane_img = draw_lanes(bird_eye, left_line, right_line, 
                         valid_left, valid_right, lane_center, center_y)
    
    info_img = add_info_overlay(lane_img, steering_angle, lane_center, 
                               valid_left, valid_right, center_y)
    
    return bird_eye, info_img, edges, steering_angle

try:
    while True:
        # Capture frame from camera
        frame = picam2.capture_array()
        
        # Use the function already defined in your script
        bird_eye, info_img, edges, steering_angle = process_frame(frame)
        
        # Show processed image
        cv2.imshow("Lane Detection", info_img)
        
        # Use angle to control steering servo
        px.set_dir_servo_angle(steering_angle)
        
        # Optionally: move forward at low speed
        px.forward(SPEED)
        
        # Exit with 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if cv2.waitKey(1) & 0xFF == ord('p'):
            SPEED = 0
        
        if cv2.waitKey(1) & 0xFF == ord('l'):
            SPEED = 1
            
except KeyboardInterrupt:
    pass

finally:
    px.stop()
    px.forward(0)
    picam2.stop()
    px.set_dir_servo_angle(0)
    cv2.destroyAllWindows()