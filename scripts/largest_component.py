#!/usr/bin/env python3
import sys
import pymeshlab

if len(sys.argv) < 3:
    print(f"Usage: {sys.argv[0]} <input.ply> <output.ply>")
    sys.exit(1)

ms = pymeshlab.MeshSet()
ms.load_new_mesh(sys.argv[1])
ms.compute_selection_by_small_disconnected_components_per_face(1)
ms.meshing_remove_selected_vertices_and_faces()
ms.save_current_mesh(sys.argv[2])
print(f"Saved largest component to {sys.argv[2]}")
