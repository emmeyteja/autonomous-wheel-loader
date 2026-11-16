"""Supported Isaac Sim URDF importer workflow, also available kitlessly."""
from importlib.metadata import version
from pathlib import Path
import json
import shutil


def convert_urdf(urdf, output, fix_base=False):
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    wheels = [side+'_wheel_joint' for side in ('rear_left', 'front_left', 'rear_right', 'front_right')]
    targets = dict.fromkeys(wheels, 'velocity') | {'arm_joint': 'position', 'bucket_joint': 'position'}
    stiffness = dict.fromkeys(wheels, 0.) | {'arm_joint': 1000., 'bucket_joint': 70.}
    damping = dict.fromkeys(wheels, .2) | {'arm_joint': 10., 'bucket_joint': .01}
    config = URDFImporterConfig(
        urdf_path=str(Path(urdf).resolve()), usd_path=str(Path(output).resolve()),
        merge_fixed_joints=True, merge_mesh=False, collision_from_visuals=False,
        collision_type='Convex Hull', allow_self_collision=True, fix_base=fix_base,
        joint_drive_type='force', joint_target_type=targets,
        override_joint_stiffness=stiffness,
        override_joint_damping=damping,
        run_asset_transformer=True, run_multi_physics_conversion=True,
    )
    result = URDFImporter(config).import_urdf()
    from pxr import Usd
    stage = Usd.Stage.Open(result)
    stage.GetDefaultPrim().GetVariantSets().GetVariantSet('Physics').SetVariantSelection('physx')
    stage.GetRootLayer().Save()
    manifest = {'importer': 'isaacsim.asset.importer.urdf.URDFImporter',
                'package_version': version('isaacsim-asset-isolated'), 'fix_base': fix_base,
                'default_physics_variant': 'physx', 'merge_fixed_joints': True, 'self_collision_requested': True,
                'collision_from_visuals': False, 'collision_type': 'Convex Hull',
                'wheel_drive_gains_SI': {'stiffness_Nm_per_rad': 0., 'damping_Nm_s_per_rad': .2},
                'warning': 'diagnostic model, not faithful Komodo geometry or validated machine physics',
                'usd_relative_path': str(Path(result).relative_to(Path(output).resolve()))}
    Path(output).mkdir(parents=True, exist_ok=True)
    for name in ('UPSTREAM_LICENSE.txt', 'preparation.json'):
        source = Path(urdf).parent/name
        if source.is_file():
            shutil.copyfile(source, Path(output)/name)
            shutil.copyfile(source, Path(result).parent/name)
    (Path(output)/'conversion.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return result
