import logging
import math
from pathlib import Path
from typing import List

from ...model import BBox
from . import BaseLabelFormat, abs2rel_rotation, rel2abs_rotation


class LaconicFormat(BaseLabelFormat):
    """Like KittiFormat, but for LIDAR so uses only
    l - along x
    w - along y
    h - along z
    (name, location[xyz], dimensions[lwh], rotation_z) fields
    in LIDAR frame (location is bottom point of bbox, not centroid)
    """
    FILE_ENDING = ".txt"

    def __init__(
        self,
        label_folder: Path,
        export_precision: int,
        relative_rotation: bool = False,
    ) -> None:
        super().__init__(label_folder, export_precision, relative_rotation)

    def import_labels(self, pcd_path: Path) -> List[BBox]:
        labels = []

        label_path = self.label_folder.joinpath(pcd_path.stem + self.FILE_ENDING)
        if label_path.is_file():
            with label_path.open("r") as read_file:
                label_lines = read_file.readlines()

            for line in label_lines:
                line_elements = line.split()
                centroid = [float(v) for v in line_elements[1:4]]
                dimensions = tuple([float(v) for v in line_elements[4:7]])
                centroid[2] += dimensions[2] / 2
                bbox = BBox(*centroid, *dimensions)
                bbox.set_rotations(0, 0, rel2abs_rotation(float(line_elements[7])))
                bbox.set_classname(line_elements[0])
                labels.append(bbox)
            logging.info("Imported %s labels from %s." % (len(label_lines), label_path))
        return labels

    def export_labels(self, bboxes: List[BBox], pcd_path: Path) -> None:
        data = str()

        # Labels
        for bbox in bboxes:
            obj_type = bbox.get_classname()
            centroid = list(bbox.get_center())
            dimensions = bbox.get_dimensions()
            centroid[2] -= dimensions[2] / 2
            location = " ".join([str(self.round_dec(v)) for v in centroid])
            dimensions_str = " ".join([str(self.round_dec(v)) for v in dimensions])
            rotation_z = bbox.get_z_rotation()
            rotation_y = self.round_dec(abs2rel_rotation(rotation_z))

            data += (
                " ".join(
                    [
                        obj_type,
                        location,
                        dimensions_str,
                        str(rotation_y),
                    ]
                )
                + "\n"
            )

        # Save to TXT
        path_to_file = self.save_label_to_file(pcd_path, data)
        logging.info(
            f"Exported {len(bboxes)} labels to {path_to_file} "
            f"in {self.__class__.__name__} formatting!"
        )
