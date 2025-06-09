import numpy as np
from scipy.signal import correlate
from typing import List, Dict, Tuple

def calculate_video_sync_delays(
    camera_signals: List[np.ndarray],
    reference_camera_index: int = 0,
    frame_rate: float = 30.0
) -> Tuple[Dict[str, float], Dict[str, str]]:
    """
    Calculates time delays for synchronizing multiple video files based on
    pre-extracted time-series signals (e.g., ArUco marker data).

    Args:
        camera_signals (List[np.ndarray]): A list where each element is a 1D NumPy array
                                          representing the time-series signal for one camera.
                                          Each array should have the same length (number of frames).
        reference_camera_index (int, optional): The index of the camera to use as the reference
                                                for delay calculations. Defaults to 0 (the first camera).
        frame_rate (float, optional): The frame rate of the video files in frames per second (fps).
                                      Used to convert frame delays to seconds. Defaults to 30.0.

    Returns:
        Tuple[Dict[str, float], Dict[str, str]]:
            A tuple containing two dictionaries:
            1. `relative_delays_seconds`: Delays of each camera relative to the reference camera, in seconds.
                                          Keys are 'Camera X', values are float seconds.
            2. `alignment_instructions`: Instructions for trimming/padding each video to align
                                        them all to a common start time (t0).
                                        Keys are 'Camera X', values are string instructions.
    
    Raises:
        ValueError: If camera_signals is empty or reference_camera_index is out of bounds.
    """

    if not camera_signals:
        raise ValueError("camera_signals list cannot be empty.")
    if not 0 <= reference_camera_index < len(camera_signals):
        raise ValueError(f"reference_camera_index {reference_camera_index} is out of bounds for {len(camera_signals)} cameras.")

    reference_signal = camera_signals[reference_camera_index]
    sequence_length = len(reference_signal)

    # Validate that all signals have the same length
    for i, signal in enumerate(camera_signals):
        if len(signal) != sequence_length:
            raise ValueError(f"Signal for Camera {i} has length {len(signal)}, but reference signal has length {sequence_length}. All signals must have the same length.")

    calculated_delays_frames: Dict[str, int] = {}
    relative_delays_seconds: Dict[str, float] = {}

    print(f"--- Calculating Delays relative to Camera {reference_camera_index} ---")

    for i, current_camera_signal in enumerate(camera_signals):
        camera_name = f'Camera {i}'

        if i == reference_camera_index:
            calculated_delays_frames[camera_name] = 0
            relative_delays_seconds[camera_name] = 0.0
            print(f"  {camera_name} (Reference): Delay = 0 frames (0.00 s)")
            continue

        # Perform cross-correlation
        correlation_output = correlate(current_camera_signal, reference_signal, mode='full')
        
        # Calculate the lags corresponding to the correlation output
        # The 'full' mode correlation output has length (len_signal_A + len_signal_B - 1)
        # The lags range from -(len_ref - 1) to (len_current - 1)
        lags = np.arange(-(sequence_length - 1), sequence_length)
        
        # Find the lag at which the cross-correlation is maximum
        delay_in_frames = lags[np.argmax(correlation_output)]
        delay_in_seconds = delay_in_frames / frame_rate
        
        calculated_delays_frames[camera_name] = delay_in_frames
        relative_delays_seconds[camera_name] = delay_in_seconds
        
        print(f"  {camera_name}: Delay = {delay_in_frames} frames ({delay_in_seconds:.2f} s)")

    print("\n--- Generating Alignment Instructions ---")

    # To align all videos to a common start (t0):
    # Find the camera that starts earliest (most negative delay).
    # This value might be 0 if the reference camera is the earliest.
    all_delay_values = list(calculated_delays_frames.values())
    min_overall_delay_frames = min(all_delay_values) 

    print(f"  Overall earliest start (relative to chosen reference): {min_overall_delay_frames} frames")

    alignment_instructions: Dict[str, str] = {}
    for cam_name, delay_frames in calculated_delays_frames.items():
        # 'shift_for_sync' is the amount of frames to TRIM (if positive) or PAD (if negative)
        # from the *original* video to align it to the common start point (t0).
        shift_for_sync = delay_frames - min_overall_delay_frames
        
        if shift_for_sync > 0:
            alignment_instructions[cam_name] = f"Trim {shift_for_sync} frames from the start"
        elif shift_for_sync < 0:
            alignment_instructions[cam_name] = f"Pad {-shift_for_sync} frames at the start (with black frames/duplicated first frame)"
        else:
            alignment_instructions[cam_name] = "No shift needed (or it's the earliest-starting camera)"

    return relative_delays_seconds, alignment_instructions

# --- Example Usage ---
if __name__ == "__main__":
    # Simulate your pre-extracted time-series signals from video processing
    # In a real scenario, you would have code here to load your videos,
    # detect markers frame-by-frame, and extract a scalar value (e.g., marker Z-translation).

    # Example: Simulate a basic movement for 500 frames
    base_movement = np.sin(np.linspace(0, 4 * np.pi, 500)) + np.linspace(0, 1, 500) * 5

    # Camera 0 (Reference) - no true delay
    signal_cam0 = base_movement + np.random.normal(0, 0.1, 500)

    # Camera 1 - lags by 30 frames
    signal_cam1 = np.roll(base_movement, 30) + np.random.normal(0, 0.1, 500)

    # Camera 2 - leads by 50 frames
    signal_cam2 = np.roll(base_movement, -50) + np.random.normal(0, 0.1, 500)

    # Camera 3 - lags by 10 frames
    signal_cam3 = np.roll(base_movement, 10) + np.random.normal(0, 0.1, 500)

    all_camera_signals_example = [signal_cam0, signal_cam1, signal_cam2, signal_cam3]

    print("--- Running Synchronization Function ---")
    relative_delays, alignment_instructions = calculate_video_sync_delays(
        all_camera_signals_example,
        reference_camera_index=0, # Let's use Camera 0 as reference
        frame_rate=30.0
    )

    print("\n--- RESULTS ---")
    print("\nCalculated Delays (relative to reference camera):")
    for cam, delay_s in relative_delays.items():
        print(f"{cam}: {delay_s:.2f} seconds")

    print("\nFinal Video Alignment Instructions (to common t0):")
    for cam, instruction in alignment_instructions.items():
        print(f"{cam}: {instruction}")

    # Example with a different reference camera
    print("\n--- Running Synchronization Function with Camera 1 as Reference ---")
    relative_delays_cam1_ref, alignment_instructions_cam1_ref = calculate_video_sync_delays(
        all_camera_signals_example,
        reference_camera_index=1, # Now using Camera 1 as reference
        frame_rate=30.0
    )
    print("\n--- RESULTS (Camera 1 Reference) ---")
    print("\nCalculated Delays (relative to Camera 1):")
    for cam, delay_s in relative_delays_cam1_ref.items():
        print(f"{cam}: {delay_s:.2f} seconds")

    print("\nFinal Video Alignment Instructions (to common t0):")
    for cam, instruction in alignment_instructions_cam1_ref.items():
        print(f"{cam}: {instruction}")