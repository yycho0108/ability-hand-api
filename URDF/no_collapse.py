#!/usr/bin/env python3

from domi.util.urdf.util import PatchedURDF


def main():
    side: str = 'right'
    src_urdf = PatchedURDF.load(F'./ability_hand_{side}_large_domi.urdf',
                                load_meshes=False)

    tip_links = [l for l in src_urdf.robot.links if ('_tip' in l.name)]
    tip_joints = [j for j in src_urdf.robot.joints if ('_tip' in j.child)]

    urdf = PatchedURDF.load(F'./ability_hand_{side}_large_domi_coacd.urdf',
                            load_meshes=False)
    urdf.robot.links.extend(tip_links)
    urdf.robot.joints.extend(tip_joints)

    xml = urdf.write_xml()
    for e in xml.findall('joint[@type="fixed"]'):
        name = e.attrib['name']
        if '_anchor' in name or name.endswith('base'):
            print('dont_collapse', name)
            e.attrib['dont_collapse'] = 'true'
    xml.write(F'./ability_hand_{side}_large_domi_coacd_ann.urdf',
              xml_declaration=True,
              pretty_print=True)


if __name__ == '__main__':
    main()
