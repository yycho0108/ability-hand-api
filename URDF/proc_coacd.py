#!/usr/bin/env python3

import re
from domi.util.urdf.util import PatchedURDF
from domi.util.urdf.merge_col import robot_coacd, CoacdConfig


def match_any(pattern, links):
    ex = re.compile(pattern)
    return any(ex.match(l) for l in links)


def main():
    side: str = 'right'

    def coacd_cfg_fn(links, mesh):
        cls = CoacdConfig
        if match_any('base', links):
            return cls(max_concavity=0.05,
                       max_chull_count=4)
        if match_any('(index|middle|ring|pinky|thumb)_(L1|L2)', links):
            return cls(max_concavity=0.02,
                       max_chull_count=2)
        return cls(max_concavity=0.05,
                   max_chull_count=4)
    urdf = PatchedURDF.load(F'./ability_hand_{side}_large_domi.urdf')
    urdf = robot_coacd(urdf, F'./coacd_{side}',
                       coacd_cfg_fn=coacd_cfg_fn)
    urdf = PatchedURDF(urdf.robot)
    urdf.write_xml_file(
        F'./ability_hand_{side}_large_domi_coacd.urdf'
    )


if __name__ == '__main__':
    main()
