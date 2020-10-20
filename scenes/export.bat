"C:\Program Files\Blender Foundation\Blender 2.90\blender.exe" -y --background --python export-scene.py -- "%1.blend":%2 "..\dist\%3.scene"

"C:\Program Files\Blender Foundation\Blender 2.90\blender.exe" -y --background --python export-meshes.py -- "%1.blend":%2 "..\dist\%3.pnct"

"C:\Program Files\Blender Foundation\Blender 2.90\blender.exe" -y --background --python export-walkmeshes.py -- "%1.blend":"WalkMeshes" "..\dist\%3.w"