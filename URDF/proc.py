#!/usr/bin/env python3

import copy
import numpy as np
from lxml import etree
from yourdfpy import URDF, Inertial
from trimesh import transformations as tf
from domi.util.urdf.util import (
    PatchedURDF,
    remove_link,
    rename_link,
    add_virtual_inertia,
    reset_mimic_origin
)


def _is_fsr(link):
    return ('fsr' in link.name)


def _rename_anchor(link):
    return link.replace('_anchor', '_tip')


def main():
    urdf = PatchedURDF.load('./ability_hand_right_large.urdf')
    urdf = remove_link(urdf, _is_fsr)
    urdf = rename_link(urdf, _rename_anchor)
    urdf = add_virtual_inertia(urdf)
    urdf = reset_mimic_origin(urdf)
    # rename_joint(urdf, 'thmb_anchor', 'thumb_anchor')
    # rename_joint(urdf, 'pnky_anchor', 'pinky_anchor')
    urdf.write_xml_file('./ability_hand_right_large_domi.urdf')


if __name__ == '__main__':
    main()
