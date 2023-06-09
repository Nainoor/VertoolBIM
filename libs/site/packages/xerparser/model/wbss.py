from xerparser.model.classes.wbs import WBS
from typing import List


class WBSs:
    _wbss = []

    def __init__(self):
        self.index = 0

    def add(self, params):
        wbs = WBS(params)
        self._wbss.append(wbs)

    @staticmethod
    def get_by_project(id) -> List[WBS]:
        return list(filter(lambda x: x.proj_id == id, WBSs._wbss))



    def __iter__(self):
        return self

    def __next__(self) -> WBS:
        if self.index >= len(self._wbss):
            raise StopIteration
        idx = self.index
        self.index +=1
        return self._wbss[idx]