import json
import random
import os

def add_noise(value, noise_factor=0.05):
    noise = random.uniform(-noise_factor, noise_factor)
    return value + noise
def modify_cones(data, change_percentage=0.05):
    modified_data = {'x': [], 'y': [], 'color': []}
    blue_count = 0
    yellow_count = 0
    for x, y, color in zip(data['x'], data['y'], data['color']):
        if color == 'yellow':
            if yellow_count < len(data['color']) * change_percentage:
                modified_data['x'].append(add_noise(x))
                modified_data['y'].append(add_noise(y))
                modified_data['color'].append('blue')
                blue_count += 1
            else:
                modified_data['x'].append(add_noise(x))
                modified_data['y'].append(add_noise(y))
                modified_data['color'].append('yellow')
            yellow_count += 1
        elif color == 'blue':
            if blue_count < len(data['color']) * change_percentage:
                modified_data['x'].append(add_noise(x))
                modified_data['y'].append(add_noise(y))
                modified_data['color'].append('yellow')
                yellow_count += 1
            else:
                modified_data['x'].append(add_noise(x))
                modified_data['y'].append(add_noise(y))
                modified_data['color'].append('blue')
            blue_count += 1
        else:  # orange cones remain unchanged
            modified_data['x'].append(add_noise(x))
            modified_data['y'].append(add_noise(y))
            modified_data['color'].append('orange')
    return modified_data


def main():
    folder_path = 'C:/Users/Jumana/Desktop/test/'
    change_percentage = 0.05  # 5% of blue and yellow cones will change color

    for filename in os.listdir(folder_path):
        if filename.endswith('.json'):
            input_file = os.path.join(folder_path, filename)
            output_file = os.path.join(folder_path, 'modified_' + filename)
            
            with open(input_file, 'r') as f:
                cones_data = json.load(f)
            
            modified_data = {
                'x': cones_data['x'],
                'y': cones_data['y'],
                'color': cones_data['color']
            }
            for i, color in enumerate(cones_data['color']):
                if color == 'blue' or color == 'yellow':
                    if random.random() < change_percentage:
                        modified_data['color'][i] = 'yellow' if color == 'blue' else 'blue'
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
