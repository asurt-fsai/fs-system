import json
import random
import os

def add_noise(value, noise_factor=0.05):
    noise = random.uniform(-noise_factor, noise_factor)
    return value + noise


def main():
    folder_path = 'C:/Users/Jumana/Desktop/FS-AI/REPO/Planning-DL-Data/Without Noise/CCW'
    output_path= 'C:/Users/Jumana/Desktop/FS-AI/REPO/Planning-DL-Data/With Noise/CCW'
    change_percentage = 0.05  # 5% of blue and yellow cones will change color

    for filename in os.listdir(folder_path):
        if filename.endswith('.json'):
            input_file = os.path.join(folder_path, filename)
            output_file = os.path.join(output_path, 'modified_' + filename)
            
            with open(input_file, 'r') as f:
                cones_data = json.load(f)
            
            modified_data = {
                'x': cones_data['x'],
                'y': cones_data['y'],
                'color': cones_data['color'],
                'x_path': cones_data['pathX'],
                'y_path': cones_data['pathY']
            }
            for i, color in enumerate(cones_data['color']):
                if color == 'blue' or color == 'yellow':
                    if random.random() < change_percentage:
                        modified_data['color'][i] = random.choice(['unknown', 'yellow']) if color == 'blue' else random.choice(['blue', 'unknown'])
                elif color == 'orange_big':
                    modified_data['color'][i] = 'orange_big'
                elif color == 'orange':
                    if random.random() < change_percentage:
                        modified_data['color'][i] = random.choice(['blue', 'yellow'])
            
            with open(output_file, 'w') as f:
                json.dump(modified_data, f, separators=(',', ':'))
            
            print("Modified cones data saved to:", output_file)

if __name__ == "__main__":
    main()
