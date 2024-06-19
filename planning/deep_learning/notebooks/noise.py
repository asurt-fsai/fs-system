"""Used for reading and writing JSON files"""
import json
import random
import os


def addNoise(value: float, noiseFactor: float = 0.05) -> float:
    """
    Adds random noise to a given value.

    Parameters:
        value (float): The original value to which noise will be added.
        noise_factor (float, optional): The range of noise to be added. Defaults to 0.05.

    Returns:
        float: The value with added noise.

    Example:
        >>> addNoise(10.0, 0.1)
        10.0432
    """
    noise = random.uniform(-noiseFactor, noiseFactor)
    return value + noise


def main() -> None:
    """
    Modifies the color of blue and yellow cones in JSON files.

    Reads JSON files from the specified folder path, modifies the color of blue and yellow cones
    based on the change percentage, and saves the modified data to the specified output path.

    Args:
        None

    Returns:
        None
    """
    folderPath = "C:/Users/Jumana/Desktop/FS-AI/REPO/Planning-DL-Data/Without Noise/CCW"
    outputPath = "C:/Users/Jumana/Desktop/FS-AI/REPO/Planning-DL-Data/With Noise/CCW"
    changePercentage = 0.05  # 5% of blue and yellow cones will change color

    for filename in os.listdir(folderPath):
        if filename.endswith(".json"):
            inputFile = os.path.join(folderPath, filename)
            outputFile = os.path.join(outputPath, "modified_" + filename)

            with open(inputFile, "r", encoding="utf-8") as file:
                conesData = json.load(file)

            modifiedData = {
                "x": conesData["x"],
                "y": conesData["y"],
                "color": conesData["color"],
                "x_path": conesData["pathX"],
                "y_path": conesData["pathY"],
            }
            for i, color in enumerate(conesData["color"]):
                if color in ("blue", "yellow"):
                    if random.random() < changePercentage:
                        modifiedData["color"][i] = (
                            random.choice(["unknown", "yellow"])
                            if color == "blue"
                            else random.choice(["blue", "unknown"])
                        )
                elif color == "orange_big":
                    modifiedData["color"][i] = "orange_big"
                elif color == "orange":
                    if random.random() < changePercentage:
                        modifiedData["color"][i] = random.choice(["blue", "yellow"])

            with open(outputFile, "w", encoding="utf-8") as file:
                json.dump(modifiedData, file, separators=(",", ":"))

            print("Modified cones data saved to:", outputFile)


if __name__ == "__main__":
    main()
