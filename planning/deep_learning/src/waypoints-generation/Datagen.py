from Trackgen import *

class DataGenerator:
    
    def __init__(self, cones_data, spline):
        self.cones_data = cones_data
        self.spline = spline
    
    # generate data
    def generate_data(self):
        
        src = {"x":self.cones_data["blue"][0], "y":self.cones_data["blue"][1], "color":[]}
        tgt = {"x":self.spline[0], "y":self.spline[1]}
        src["color"] = ["blue"] * len(src["x"])
        src["x"].extend(self.cones_data["yellow"][0])
        src["y"].extend(self.cones_data["yellow"][1])
        src["color"].extend(["yellow"] * len(self.cones_data["yellow"][0]))
        src["x"].extend(self.cones_data["orange"][0])
        src["y"].extend(self.cones_data["orange"][1])
        src["color"].extend(["orange"] * len(self.cones_data["orange"][0]))
            
        return pd.DataFrame(src,columns=None), pd.DataFrame(tgt, columns=None)
    
    
    def oneHotEncode(self,src):
        return pd.get_dummies(src, columns=["color"])    
    
    def ToTensors(self,src,tgt):
        src = {column: torch.tensor(src[column].values, dtype=torch.float32) for column in src.columns}
        tgt = {column: torch.tensor(tgt[column].values, dtype=torch.float32) for column in tgt.columns}
        src = torch.cat([src["x"].view(-1,1), src["y"].view(-1,1), src["color_blue"].view(-1,1), src["color_orange"].view(-1,1), src["color_yellow"].view(-1,1)], dim=1)
        tgt = torch.cat([tgt["x"].view(-1,1), tgt["y"].view(-1,1)], dim=1)
        return src, tgt