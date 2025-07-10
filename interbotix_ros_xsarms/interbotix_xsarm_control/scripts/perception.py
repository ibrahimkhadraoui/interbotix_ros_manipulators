class SimPerception:
    def __init__(self):
        self.object_positions = {
            "spatula": (-0.35, 0.047, 1.042),
            "cup": (-0.4, 0.18, 1.1),
            "plate": (-0.13, 0.13, 1.15),
            "basket": (0.20, -0.26, 1.32),
            "scissors": (0.13, 0.14, 1.03),
            "sink": (0, 0.51, 1.06),
            "top_right": (-0.4, -0.26, 1.05),
            "top_left": (-0.4, 0.26, 1.05),
            "bottom_right": (0.4, -0.26, 0.95),
            "bottom_left": (0.4, 0.26, 0.95),
            "napkin": (0.32, 0.13, 1.15),
        }

    def get(self, object_name):
        return self.object_positions.get(object_name)