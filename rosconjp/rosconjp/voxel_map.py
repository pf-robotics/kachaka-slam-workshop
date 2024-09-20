from dataclasses import dataclass

import numpy as np


@dataclass
class Cell:
    point2d: np.ndarray
    count: int


class VoxelMap:
    def __init__(self, resolution: float) -> None:
        self._resolution = resolution
        self._dict: dict[int, Cell] = {}

    def index(self, p: np.ndarray) -> int:
        indices = np.ceil(p / self._resolution)
        return hash((indices[0], indices[1]))

    def add(self, p: np.ndarray) -> None:
        i = self.index(p)
        if i not in self._dict:
            self._dict[i] = Cell(np.copy(p), 1)
        else:
            cell = self._dict[i]
            self._dict[i].point2d = (cell.point2d * cell.count + p) / (cell.count + 1)
            self._dict[i].count += 1

    def get(self, p: np.ndarray) -> Cell | None:
        i = self.index(p)
        return self._dict.get(i, None)

    @property
    def voxel_map(self) -> dict[int, Cell]:
        return self._dict
