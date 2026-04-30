import numpy as np

def load_data(file_path):
    return np.loadtxt(file_path, delimiter='\t')

def apply_calibration(data, translation, transform_matrix):
    centered = data - translation
    scaled = centered @ transform_matrix
    return scaled

def save_data(data, output_file):
    """Save 3D points to a text file."""
    np.savetxt(output_file, data, delimiter='\t', fmt='%.6f')

def process_with_manual_calibration(input_file, output_file, translation, transform_matrix):
    raw_data = load_data(input_file)
    calibrated_data = apply_calibration(raw_data, translation, transform_matrix)
    save_data(calibrated_data, output_file)

    print(f"✅ Applied manual calibration to '{input_file}' → '{output_file}'")
    print(f"Translation vector:\n{translation}")
    print(f"Transformation matrix:\n{transform_matrix}")

# Example usage
if __name__ == "__main__":
    input_file = "magnetometer_data.txt"
    output_file = "sphere_data.txt"

    # Replace these with your actual calibration values from the external tool
    translation_vector = np.array([
        0.019428,  # X offset
        0.009191,  # Y offset
        0.046580    # Z offset
    ])

    transformation_matrix = np.array([
        [1.159403, 0.011832, -0.000098],
        [0.011832, 1.171830, 0.000800],
        [-0.000098, 0.000800, 1.211644]
    ])

    process_with_manual_calibration(input_file, output_file, translation_vector, transformation_matrix)